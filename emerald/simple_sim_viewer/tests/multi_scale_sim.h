#pragma once

#include <memory>
#include <variant>
#include <iterator>
#include <algorithm>

namespace emerald::simple_sim_viewer {

using V2ll = Imath::Vec2<int64_t>;
using Box2ll = Imath::Box<V2ll>;

template <typename Iterator>
struct KdTreeNode2 {
    struct Branch {
        std::unique_ptr<KdTreeNode2<Iterator>> left;
        std::unique_ptr<KdTreeNode2<Iterator>> right;
    };

    struct Leaf {
        Iterator begin;
        Iterator end;
    };

    std::variant<Branch, Leaf> body;
    Box2ll bounds;

    template <typename Sort_point_function, typename Bounds_function>
    KdTreeNode2(Sort_point_function&& sort_pt_function,
                Bounds_function&& bounds_function,
                Iterator begin,
                Iterator end,
                int const max_sub_divs,
                int const max_per_leaf) {
        bounds.makeEmpty();
        for (auto iter = begin; iter != end; ++iter) {
            bounds.extendBy(bounds_function(*iter));
        }

        auto const N = std::distance(begin, end);

        if (max_sub_divs <= 0 || N <= max_per_leaf) {
            body.emplace<Leaf>(begin, end);
        } else {
            auto mid = begin + N / 2;

            auto const bounds_size = bounds.size();
            if (bounds_size.x > bounds_size.y) {
                std::nth_element(
                    begin,
                    min,
                    end,
                    [sort_pt_func = std::forward<Sort_point_function>(
                         sort_pt_function)](auto const a, auto const b) {
                        return sort_pt_func(a).x < sort_pt_func(b).x;
                    });
            } else {
                std::nth_element(
                    begin,
                    min,
                    end,
                    [sort_pt_func = std::forward<Sort_point_function>(
                         sort_pt_function)](auto const a, auto const b) {
                        return sort_pt_func(a).y < sort_pt_func(b).y;
                    });
            }

            Branch branch;
            branch.left = std::make_unique<KdTreeNode2<Iterator>>(
                std::forward<Sort_point_function>(sort_pt_function),
                std::forward<Bounds_function>(bounds_function),
                begin,
                mid,
                max_sub_divs - 1,
                max_per_leaf);

            branch.right = std::make_unique<KdTreeNode2<Iterator>>(
                std::forward<Sort_point_function>(sort_pt_function),
                std::forward<Bounds_function>(bounds_function),
                mid,
                end,
                max_sub_divs - 1,
                max_per_leaf);

            body.emplace<Branch>(std::move(branch));
        }
    }

    KdTreeNode2(KdTreeNode2 const&) = delete;
    KdTreeNode2& operator=(KdTreeNode2 const&) = delete;
    KdTreeNode2(KdTreeNode2&&) = default;
    KdTreeNode2& operator=(KdTreeNode2&&) = default;
    ~KdTreeNode2 = default;

    template <typename Bounds_test_function,
              typename Leaf_visitor_function>
    void visit(Bounds_test_function&& bounds_test,
               Leaf_visitor_function&& leaf_visit) const {
        if (!bounds_test(bounds)) {
            return;
        }

        if (body.holds_alternative<Leaf>) {
            auto const& leaf = std::get<Leaf>(body);
            std::for_each(leaf.begin, leaf.end, std::forward<Leaf_visitor_function>(leaf_visit));
        } else {
            auto const& branch = std::get<Branch>(body);
            branch.left->visit(std::forward<Bounds_test_function>(bounds_test),
                               std::forward<Leaf_visitor_function>(leaf_visit));
            branch.right->visit(std::forward<Bounds_test_function>(bounds_test),
                               std::forward<Leaf_visitor_function>(leaf_visit));
        }
    }
};

struct IndexedPointKdTree2ll {
    std::vector<size_t> indices;
    std::unique_ptr<KdTreeNode2<std::vector<size_t>::iterator>> top_node;

    explicit IndexedPointKdTree2ll(std::vector<V2ll> const& position) {
        indices.resize(position.size());
        std::iota(indices.begin(), indices.end(), 0);

        auto const point_func = [&position](size_t const i) -> V2ll const& { return position[i]; };

        top_node = std::make_unique<KdTreeNode2<std::vector<size_t>::iterator>>(
                                                                                point_func,
                                                                                point_func,
                                                                                indices.begin(),
                                                                                indices.end(),
                                                                                16,
                                                                                8);
    }
};

struct Single_scale_sim {
    std::vector<V2ll> position;
    std::vector<C4c> rgba;

    uint8_t diameter_bit_shift;

};

}  // namespace emerald::simple_sim_viewer