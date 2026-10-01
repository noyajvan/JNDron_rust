//! The two ring buffers used by the original firmware: `status_queue`
//! (outgoing STATUS_TEXT, drained 5-at-a-time every 500 ms) and
//! `reason_queue` (incoming WARNING-or-worse STATUSTEXT from the FC).

/// A fixed-size ring of short text lines, mirroring the C++
/// `char queue[N][72]` arrays with `write`/`read` cursors.
#[derive(Debug, Clone)]
pub struct TextRing<const N: usize> {
    slots: [String; N],
    write: usize,
    read: usize,
}

impl<const N: usize> Default for TextRing<N> {
    fn default() -> Self {
        TextRing {
            slots: std::array::from_fn(|_| String::new()),
            write: 0,
            read: 0,
        }
    }
}

impl<const N: usize> TextRing<N> {
    pub fn new() -> Self {
        Self::default()
    }

    /// Push a line, truncating to 71 bytes like `strncpy(..., 71)` did.
    /// If the buffer is full the oldest entry is dropped (as in C++).
    pub fn push(&mut self, text: &str) {
        let next = (self.write + 1) % N;
        if next == self.read {
            self.read = (self.read + 1) % N;
        }
        self.slots[self.write] = truncate71(text);
        self.write = next;
    }

    pub fn is_empty(&self) -> bool {
        self.read == self.write
    }

    pub fn len(&self) -> usize {
        if self.write >= self.read {
            self.write - self.read
        } else {
            N - self.read + self.write
        }
    }

    /// Pop the oldest line.
    pub fn pop(&mut self) -> Option<String> {
        if self.is_empty() {
            return None;
        }
        let s = std::mem::take(&mut self.slots[self.read]);
        self.read = (self.read + 1) % N;
        Some(s)
    }

    pub fn clear(&mut self) {
        self.read = 0;
        self.write = 0;
        for s in self.slots.iter_mut() {
            s.clear();
        }
    }

    /// True if any buffered line contains `needle` (used by tests).
    pub fn contains(&self, needle: &str) -> bool {
        let mut i = self.read;
        while i != self.write {
            if self.slots[i].contains(needle) {
                return true;
            }
            i = (i + 1) % N;
        }
        false
    }
}

/// Truncate to at most 71 bytes on a UTF-8 char boundary (all our text is
/// ASCII, so this is a straight byte cut).
fn truncate71(text: &str) -> String {
    let b = text.as_bytes();
    let n = b.len().min(71);
    String::from_utf8_lossy(&b[..n]).into_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_pop_fifo() {
        let mut q: TextRing<4> = TextRing::new();
        q.push("a");
        q.push("b");
        q.push("c");
        assert_eq!(q.len(), 3);
        assert_eq!(q.pop().as_deref(), Some("a"));
        assert_eq!(q.pop().as_deref(), Some("b"));
        assert_eq!(q.pop().as_deref(), Some("c"));
        assert_eq!(q.pop(), None);
        assert!(q.is_empty());
    }

    #[test]
    fn overflow_drops_oldest() {
        // N slots, but only N-1 usable (one slot is the separator), exactly
        // like the C++ `(w+1)%N == r` full check.
        let mut q: TextRing<4> = TextRing::new();
        q.push("a");
        q.push("b");
        q.push("c");
        q.push("d"); // overwrites "a"
        assert_eq!(q.len(), 3);
        assert_eq!(q.pop().as_deref(), Some("b"));
        assert_eq!(q.pop().as_deref(), Some("c"));
        assert_eq!(q.pop().as_deref(), Some("d"));
    }

    #[test]
    fn truncates_to_71_bytes() {
        let mut q: TextRing<2> = TextRing::new();
        let long = "x".repeat(200);
        q.push(&long);
        assert_eq!(q.pop().unwrap().len(), 71);
    }
}
