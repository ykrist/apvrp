use std::hash::{Hasher, Hash, BuildHasher};

#[derive(Debug, Copy, Clone)]
struct UnorderedPair<T>(T, T);

#[derive(Debug, Copy, Clone)]
struct UnorderedRefPair<'a, T>(&'a T, &'a T);

impl<T: Hash> Hash for UnorderedPair<T> {
  fn hash<H: Hasher>(&self, state: &mut H) {
    let mut hash1 = fnv::FnvBuildHasher::default().build_hasher();
    self.0.hash(&mut hash1);
    let mut hash2 = fnv::FnvBuildHasher::default().build_hasher();
    self.1.hash(&mut hash2);
    state.write_u64(hash1.finish() ^ hash2.finish())
  }
}
