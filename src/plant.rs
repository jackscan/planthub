use defmt::{error, info};
use embassy::util::Signal;
use futures_intrusive::sync::{LocalMutex, LocalMutexGuard};

pub type Id = u8;
pub type Stamp = u8;
pub type StampedInfo = (Stamp, Info);

pub struct StatusList<'a> {
    sig: Signal<()>,
    list: LocalMutex<StampedList<'a>>,
}

pub struct StampedList<'a> {
    data: &'a mut [StampedInfo],
    size: usize,
    stamp: Stamp,
}

pub struct Iter<'i, 'a: 'i> {
    list: LocalMutexGuard<'i, StampedList<'a>>,
    index: usize,
    lower: Stamp,
    upper: Stamp,
}

#[derive(Clone, Copy, defmt::Format)]
#[repr(C)]
pub struct Info {
    pub id: Id,
    pub temperature: i8,
    pub weight: u16,
}

impl Info {
    pub const fn new(id: Id, temperature: i8, weight: u16) -> Self {
        Self {
            id,
            temperature,
            weight,
        }
    }
}

impl<'a> StampedList<'a> {
    fn remove_old(&mut self, max_age: Stamp) {
        let mut i = self.size;
        while i > 0 {
            i -= 1;
            if (self.stamp - self.data[i].0) > max_age {
                info!("stamp: {}, removing old {}", self.stamp, self.data[i]);
                self.size -= 1;
                if i != self.size {
                    self.data[i] = self.data[self.size];
                }
            }
        }
    }
}

impl<'a> StatusList<'a> {
    pub fn new(list: &'a mut [StampedInfo]) -> Self {
        Self {
            sig: Signal::new(),
            list: LocalMutex::new(
                StampedList {
                    data: list,
                    size: 0,
                    stamp: 0,
                },
                false,
            ),
        }
    }

    pub async fn update(&self, info: Info) {
        let max_age = 2;

        let mut list = self.list.lock().await;

        list.remove_old(max_age);

        if let Some(idx) = list.data[..list.size]
            .iter()
            .position(|e| e.1.id == info.id)
            .or_else(|| {
                if list.size < list.data.len() {
                    list.size += 1;
                    Some(list.size - 1)
                } else {
                    None
                }
            })
        {
            info!("update {} of {}:  {}", idx, list.size, info);
            list.data[idx] = (list.stamp, info);
            self.sig.signal(());
        } else {
            error!("no space for {}", info);
        }
    }

    pub async fn next_update<'i>(&'i self, stamp: &mut Option<Stamp>) -> Iter<'i, 'a> {
        if stamp.is_some() {
            self.sig.wait().await;
        }

        let mut list = self.list.lock().await;

        if stamp.is_some() {
            list.stamp += 1;
        }

        let lower = stamp.unwrap_or(list.stamp + 1);
        let upper = list.stamp;
        *stamp = Some(upper);
        let iter = Iter {
            list,
            index: 0,
            lower,
            upper,
        };

        iter
    }
}

impl<'i, 'a: 'i> Iterator for Iter<'i, 'a> {
    type Item = Info;
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(i) = self.list.data[self.index..self.list.size]
            .iter()
            .position(|&(u, _)| {
                (self.lower <= u && u < self.upper)
                    || (self.lower > self.upper && (self.lower <= u || u < self.upper))
            })
        {
            self.index += i + 1;
            Some(self.list.data[self.index - 1].1)
        } else {
            self.index = self.list.size;
            None
        }
    }
}
