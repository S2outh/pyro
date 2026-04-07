use core::cmp::Ordering;
use heapless::Vec;

pub trait Sortable<T> {
    fn sort_by<F: FnMut(&T, &T) -> Ordering>(&mut self, cmp: F);
}

impl<T, const N: usize> Sortable<T> for Vec<T, N> {
    fn sort_by<F: FnMut(&T, &T) -> Ordering>(&mut self, mut cmp: F) {
        for i in 0..self.len() {
            for j in i + 1..self.len() {
                if cmp(&self[i], &self[j]) == Ordering::Greater {
                    self.swap(i, j);
                }
            }
        }
    }
}
