#pragma once

#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

namespace p2 {

/**
 * Min-Fibonacci heap: smallest key is on top.
 * Amortized O(1) insert and decrease_key; O(log n) extract_min.
 * High constant factors vs binary heaps — useful for theory and comparison.
 */
template <typename Key, typename T>
class FibonacciHeap {
    struct Node {
        Key key{};
        T value{};
        Node* parent = nullptr;
        Node* child = nullptr;
        Node* left = nullptr;
        Node* right = nullptr;
        int degree = 0;
        bool mark = false;
    };

public:
    class Handle {
        friend class FibonacciHeap;

    public:
        Handle() = default;
        [[nodiscard]] bool valid() const noexcept { return node_ != nullptr; }

    private:
        explicit Handle(Node* n) : node_(n) {}
        Node* node_ = nullptr;
    };

    FibonacciHeap() = default;
    FibonacciHeap(const FibonacciHeap&) = delete;
    FibonacciHeap& operator=(const FibonacciHeap&) = delete;

    FibonacciHeap(FibonacciHeap&& other) noexcept : min_(other.min_), n_(other.n_) {
        other.min_ = nullptr;
        other.n_ = 0;
    }

    FibonacciHeap& operator=(FibonacciHeap&& other) noexcept {
        if (this == &other)
            return *this;
        clear();
        min_ = other.min_;
        n_ = other.n_;
        other.min_ = nullptr;
        other.n_ = 0;
        return *this;
    }

    ~FibonacciHeap() { clear(); }

    [[nodiscard]] bool empty() const noexcept { return min_ == nullptr; }
    [[nodiscard]] std::size_t size() const noexcept { return n_; }

    Handle insert(Key key, T value) {
        Node* x = new Node;
        x->key = std::move(key);
        x->value = std::move(value);
        x->parent = nullptr;
        x->child = nullptr;
        x->degree = 0;
        x->mark = false;
        x->left = x->right = x;
        add_to_root_ring(x);
        ++n_;
        return Handle(x);
    }

    std::pair<Key, T> extract_min() {
        assert(!empty());
        Node* z = min_;

        if (z->child != nullptr) {
            Node* c = z->child;
            Node* start = c;
            do {
                c->parent = nullptr;
                c->mark = false;
                c = c->right;
            } while (c != start);
            merge_rings_at(z, start);
            z->child = nullptr;
        }

        if (z->right == z) {
            min_ = nullptr;
        } else {
            Node* r = z->right;
            unlink_from_ring(z);
            min_ = r;
        }

        --n_;
        std::pair<Key, T> out{std::move(z->key), std::move(z->value)};
        delete z;

        if (min_ != nullptr)
            consolidate();

        return out;
    }

    void decrease_key(Handle h, Key new_key) {
        assert(h.valid());
        Node* x = h.node_;
        assert(!(x->key < new_key) && "new_key must be <= old key for min-heap");
        if (!(new_key < x->key) && !(x->key < new_key))
            return;
        x->key = std::move(new_key);

        Node* y = x->parent;
        if (y != nullptr && x->key < y->key) {
            cut(x, y);
            cascading_cut(y);
        }
        if (min_ != nullptr && x->key < min_->key)
            min_ = x;
    }

    [[nodiscard]] const Key& min_key() const {
        assert(min_ != nullptr);
        return min_->key;
    }

private:
    static void merge_rings_at(Node* a, Node* b) {
        Node* al = a->left;
        Node* bl = b->left;
        a->left = bl;
        bl->right = a;
        b->left = al;
        al->right = b;
    }

    void add_to_root_ring(Node* x) {
        if (min_ == nullptr) {
            min_ = x;
            x->left = x->right = x;
            return;
        }
        x->left = x->right = x;
        merge_rings_at(min_, x);
        if (x->key < min_->key)
            min_ = x;
    }

    static void unlink_from_ring(Node* x) {
        x->left->right = x->right;
        x->right->left = x->left;
        x->left = x->right = x;
    }

    static void link_child(Node* y, Node* x) {
        unlink_from_ring(y);
        y->parent = x;
        if (x->child == nullptr) {
            x->child = y;
            y->left = y->right = y;
        } else {
            y->left = y->right = y;
            merge_rings_at(x->child, y);
        }
        ++x->degree;
        y->mark = false;
    }

    void cut(Node* x, Node* y) {
        if (y->child == x) {
            if (x->right != x)
                y->child = x->right;
            else
                y->child = nullptr;
        }
        unlink_from_ring(x);
        y->degree--;
        x->parent = nullptr;
        x->mark = false;
        add_to_root_ring(x);
    }

    void cascading_cut(Node* y) {
        Node* z = y->parent;
        if (z != nullptr) {
            if (!y->mark) {
                y->mark = true;
            } else {
                cut(y, z);
                cascading_cut(z);
            }
        }
    }

    void consolidate() {
        const int D = 64;
        std::vector<Node*> A(static_cast<std::size_t>(D), nullptr);

        if (min_ == nullptr)
            return;

        std::vector<Node*> roots;
        Node* cur = min_;
        Node* start = cur;
        do {
            roots.push_back(cur);
            cur = cur->right;
        } while (cur != start);

        for (Node* w : roots) {
            unlink_from_ring(w);
            w->left = w->right = w;
        }

        for (Node* w : roots) {
            Node* x = w;
            int d = x->degree;
            while (d < D && A[static_cast<std::size_t>(d)] != nullptr) {
                Node* y = A[static_cast<std::size_t>(d)];
                A[static_cast<std::size_t>(d)] = nullptr;
                if (y->key < x->key)
                    std::swap(x, y);
                link_child(y, x);
                d = x->degree;
            }
            if (d < D)
                A[static_cast<std::size_t>(d)] = x;
        }

        min_ = nullptr;
        for (int i = 0; i < D; ++i) {
            Node* nd = A[static_cast<std::size_t>(i)];
            if (nd != nullptr) {
                if (min_ == nullptr) {
                    nd->left = nd->right = nd;
                    min_ = nd;
                } else {
                    nd->left = nd->right = nd;
                    merge_rings_at(min_, nd);
                    if (nd->key < min_->key)
                        min_ = nd;
                }
            }
        }
    }

    static void free_subheap_ring(Node* start) {
        if (start == nullptr)
            return;
        Node* x = start;
        do {
            Node* nxt = x->right;
            free_subheap_ring(x->child);
            delete x;
            x = nxt;
        } while (x != start);
    }

    void clear() {
        if (min_ != nullptr) {
            free_subheap_ring(min_);
            min_ = nullptr;
            n_ = 0;
        }
    }

    Node* min_ = nullptr;
    std::size_t n_ = 0;
};

}  // namespace p2
