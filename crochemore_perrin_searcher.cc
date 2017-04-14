// Copyright (c) 2017 Ed Schouten <ed@nuxi.nl>
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.

#define _WITH_DPRINTF
#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>

// std::search version of the Two-Way string searching algorithm.
//
// http://www-igm.univ-mlv.fr/~mac/Articles-PDF/CP-1991-jacm.pdf
//
// This class implements the algorithm as provided in figure 21 of the
// original paper. As the computation of the maximal suffix of the
// pattern requires inequality, this class makes use of std::less, as
// opposed to using std::equal_to. When matching, std::less is invoked
// twice to match for equality.
template <class PatternIterator, class LessPredicate = std::less<>>
class crochemore_perrin_searcher {
private:
  // Offset within the pattern.
  typedef typename std::iterator_traits<PatternIterator>::difference_type xpos;

  // BinaryPredicate that computes equality, built on top of an existing
  // less-than predicate.
  class equal_to {
  public:
    constexpr equal_to(LessPredicate less) : less_(less) {}

    template <typename T>
    constexpr bool operator()(const T &lhs, const T &rhs) const {
      return !less_(lhs, rhs) && !less_(rhs, lhs);
    }

  private:
    LessPredicate less_;
  };

  // BinaryPredicate that computes greater-than, built on top of an
  // existing less-than predicate.
  class greater {
  public:
    constexpr greater(LessPredicate less) : less_(less) {}

    template <typename T>
    constexpr bool operator()(const T &lhs, const T &rhs) const {
      return less_(rhs, lhs);
    }

  private:
    LessPredicate less_;
  };

  PatternIterator x_; // Pattern.
  equal_to equal_to_; // Comparison operator.
  xpos n_;            // Length of the pattern.
  xpos l_;            // Critical position.
  xpos p_;            // The period of the pattern.
  bool is_periodic_;  // Pattern is fully periodic.

  // Maximal-Suffix() algorithm from figure 17, used to compute the
  // values of l_ and p_.
  template <typename Predicate>
  static constexpr std::pair<xpos, xpos>
  maximal_suffix(PatternIterator x, xpos n, Predicate pred) {
    xpos i = 0, j = 1, k = 0, p = 1;
    while (j + k < n) {
      const auto &a = x[j + k];
      const auto &ap = x[i + k];
      if (pred(a, ap)) {
        j += k + 1;
        k = 0;
        p = j - i;
      } else if (pred(ap, a)) {
        i = j++;
        k = 0;
        p = 1;
      } else if (k == p) {
        j += p;
        k = 0;
      } else {
        ++k;
      }
    }
    return {i, p};
  }

public:
  // Small-Period() algorithm from figure 19.
  constexpr crochemore_perrin_searcher(PatternIterator x, PatternIterator x_end,
                                       LessPredicate less = LessPredicate())
      : x_(x), n_(x_end - x), equal_to_(less) {
    auto s = std::max(maximal_suffix(x_, n_, less),
                      maximal_suffix(x_, n_, greater(less)));
    l_ = s.first;
    p_ = s.second;

    is_periodic_ = true;
    for (xpos i = 0; i < l_; ++i) {
      if (!equal_to_(x_[i], x_[p_ + i])) {
        is_periodic_ = false;
        break;
      }
    }
  }

  template <class DataIterator>
  constexpr std::pair<DataIterator, DataIterator>
  operator()(DataIterator t, DataIterator t_end) const {
    if (is_periodic_) {
      // Positions() algorithm from figure 8.
      xpos s = 0;
      while (n_ <= t_end - t) {
        auto i = std::max(l_, s);
        while (i < n_ && equal_to_(x_[i], t[i]))
          ++i;
        if (i < n_) {
          t += std::max(i - l_, s - p_) + 1;
          s = 0;
        } else {
          auto j = l_;
          while (j > s && equal_to_(x_[j - 1], t[j - 1]))
            --j;
          if (j <= s)
            return {t, t + n_};
          t += p_;
          s = n_ - p_;
        }
      }
    } else {
      // Positions-Bis() algorithm from figure 20.
      auto q = std::max(l_, n_ - l_) + 1;
      while (n_ <= t_end - t) {
        auto i = l_;
        while (i < n_ && equal_to_(x_[i], t[i]))
          ++i;
        if (i < n_) {
          t += i - l_ + 1;
        } else {
          auto j = l_;
          while (j > 0 && equal_to_(x_[j - 1], t[j - 1]))
            --j;
          if (j == 0)
            return {t, t + n_};
          t += q;
        }
      }
    }
    return {t_end, t_end};
  }
};

#ifdef __CloudABI__
#include <string_view>

static constexpr std::string_view x("Hello Hello");
static constexpr crochemore_perrin_searcher<const char *> y(x.begin(),
                                                            x.begin() + 5);

static_assert(y(x.begin(), x.end()).first == x.begin());
static_assert(y(x.begin(), x.end()).second == x.begin() + 5);

static_assert(y(x.begin() + 1, x.end()).first == x.begin() + 6);
static_assert(y(x.begin() + 1, x.end()).second == x.begin() + 11);
#endif

// Testing utility.

#if 0
int main() {
  char x[] = "abaaaba";
  crochemore_perrin_searcher<char *>(x, x + sizeof(x) - 1);
}
#elif 1
static void fill_random(char *buf, size_t buflen) {
  arc4random_buf(buf, buflen);
  for (size_t i = 0; i < buflen; ++i)
    buf[i] = 'A' + (buf[i] & 0x3);
}

int main() {
  for (size_t k = 0;; ++k) {
    dprintf(1, "ITER %zu\n", k);

    char haystack[4096];
    fill_random(haystack, sizeof(haystack));

    for (int i = 0; i < 10000; ++i) {
      char needle[4097];
      size_t needlelen = arc4random_uniform(sizeof(needle) + 1);
      fill_random(needle, needlelen);

      char *p1 = (char *)memmem((char *)haystack, sizeof(haystack),
                                (char *)needle, needlelen);
      auto p2 = crochemore_perrin_searcher<char *>(needle, needle + needlelen)(
          haystack, haystack + sizeof(haystack));
      if (needlelen == 0) {
        assert(p2.first == haystack);
        assert(p2.second == haystack);
      } else if (p1 == nullptr) {
        assert(p2.first == haystack + sizeof(haystack));
        assert(p2.second == haystack + sizeof(haystack));
      } else {
        assert(p2.first == p1);
        assert(p2.second == p1 + needlelen);
      }
    }
  }
}
#else
const char *my_memmem(const char *big, size_t biglen, const char *little,
                      size_t littlelen) {
  return crochemore_perrin_searcher<const char *>(little, little + littlelen)(
             big, big + biglen)
      .first;
}
#endif
