#pragma once

#include "common/macros.h"
#include "geometry/bitset_wrapper.h"
#include "geometry/box.h"
#include "geometry/distance.h"
#include "geometry/envelope.h"

#include <math/concepts/basic.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace sdc::geometry {

    struct BoundingBox {
        int min_x;
        int min_y;
        int max_x;
        int max_y;
    };

    template <typename ObjectsFilter>
    class SliceT {
    public:
        // grid_cell_freq = 1 / grid_cell_size
        SliceT(const geometry::Box& objects_bounding_box, double grid_cell_freq) noexcept
            : objects_bounding_box_{
                  objects_bounding_box == geometry::Box::Endless()
                      ? geometry::Box::Zero()
                      : objects_bounding_box}
            , grid_cell_freq_{grid_cell_freq}
            , height_{GetGridYIndexByY(objects_bounding_box_.maxCorner().y()) + 1}
            , width_{GetGridXIndexByX(objects_bounding_box_.maxCorner().x()) + 1}
            , slice_objects_sets_(width_ * height_, ObjectsFilter())
            , prefix_submatrix_(width_ * height_)
            , suffix_submatrix_(width_ * height_)
        {
            VERIFY(width_ >= 1 && height_ >= 1);
        }

        template <typename RequestGeometryType>
        ALWAYS_INLINE inline ObjectsFilter GetObjectsMaskForGeometry(
            const RequestGeometryType& geometry, double request_radius) const noexcept {
            VERIFY(are_submatrices_calculated_);
            const auto request_box = geometry::Envelope<geometry::Box>(geometry).expanded(request_radius);
            if (!objects_bounding_box_.isIntersects(request_box)) {
                return ObjectsFilter();
            }

            const BoundingBox grid_box = TransformBoxToBoundingBox(request_box);
            return GetFilteredSubmatrixFor(grid_box.min_x, grid_box.min_y, grid_box.max_x, grid_box.max_y);
        }

        ALWAYS_INLINE bool IsFilterFull(
            const ObjectsFilter& object_filter) const noexcept {
            return (object_filter & GetAllSliceObjects()) == object_filter;
        }

        ALWAYS_INLINE const ObjectsFilter& GetAllSliceObjects() const noexcept {
            VERIFY(are_submatrices_calculated_);
            return all_slice_objects_;
        }

        template <typename GeometryType>
        ALWAYS_INLINE void AddObjectToSlice(
            const size_t object_index, GeometryType&& object) noexcept {
            VERIFY(!are_submatrices_calculated_);
            all_slice_objects_.add(object_index);
            const BoundingBox grid_bounding_box(
                TransformBoxToBoundingBox(geometry::Envelope<geometry::Box>(object)));
            for (int i = grid_bounding_box.min_x; i <= grid_bounding_box.max_x; ++i) {
                for (int j = grid_bounding_box.min_y; j <= grid_bounding_box.max_y; ++j) {
                    slice_objects_sets_[i * height_ + j].add(object_index);
                }
            }
        }

        ALWAYS_INLINE void CalculateFilterSubmatrices() noexcept {
            VERIFY(!are_submatrices_calculated_);

            prefix_submatrix_[0] = slice_objects_sets_[0];
            for (int i = 1; i < width_; ++i) {
                prefix_submatrix_[i * height_] =
                    prefix_submatrix_[(i - 1) * height_] |
                    slice_objects_sets_[i * height_];
            }
            for (int j = 1; j < height_; ++j) {
                prefix_submatrix_[j] =
                    prefix_submatrix_[j - 1] | slice_objects_sets_[j];
            }
            for (int i = 1; i < width_; ++i) {
                for (int j = 1; j < height_; ++j) {
                    prefix_submatrix_[i * height_ + j] =
                        prefix_submatrix_[(i - 1) * height_ + j] |
                        prefix_submatrix_[i * height_ + j - 1] |
                        slice_objects_sets_[i * height_ + j];
                }
            }

            suffix_submatrix_[width_ * height_ - 1] =
                slice_objects_sets_[width_ * height_ - 1];
            if (width_ >= 2) {
                for (int i = width_ - 2; i >= 0; --i) {
                    suffix_submatrix_[i * height_ + height_ - 1] =
                        suffix_submatrix_[(i + 1) * height_ + height_ - 1] |
                        slice_objects_sets_[i * height_ + height_ - 1];
                }
            }
            if (height_ >= 2) {
                for (int j = height_ - 2; j >= 0; --j) {
                    suffix_submatrix_[(width_ - 1) * height_ + j] =
                        suffix_submatrix_[(width_ - 1) * height_ + j + 1] |
                        slice_objects_sets_[(width_ - 1) * height_ + j];
                }
            }
            if (width_ >= 2 && height_ >= 2) {
                for (int i = width_ - 2; i >= 0; --i) {
                    for (int j = height_ - 2; j >= 0; --j) {
                        suffix_submatrix_[i * height_ + j] =
                            suffix_submatrix_[(i + 1) * height_ + j] |
                            suffix_submatrix_[i * height_ + j + 1] |
                            slice_objects_sets_[i * height_ + j];
                    }
                }
            }

            slice_objects_sets_.clear();
            are_submatrices_calculated_ = true;
        }

        auto GetObjectsBoundingBox() const noexcept {
            return objects_bounding_box_;
        }
        auto GetWidth() const noexcept {
            return width_;
        }
        auto GetHeight() const noexcept {
            return height_;
        }

    private:
        ALWAYS_INLINE BoundingBox
        TransformBoxToBoundingBox(const geometry::Box& bounding_box) const noexcept {
            return BoundingBox{
                std::max(GetGridXIndexByX(bounding_box.minCorner().x()), 0),
                std::max(GetGridYIndexByY(bounding_box.minCorner().y()), 0),
                std::min(GetGridXIndexByX(bounding_box.maxCorner().x()), width_ - 1),
                std::min(GetGridYIndexByY(bounding_box.maxCorner().y()), height_ - 1)};
        }

        ALWAYS_INLINE int GetGridXIndexByX(const double x) const noexcept {
            return static_cast<int>((x - objects_bounding_box_.minCorner().x()) * grid_cell_freq_);
        }

        ALWAYS_INLINE int GetGridYIndexByY(const double y) const noexcept {
            return static_cast<int>((y - objects_bounding_box_.minCorner().y()) * grid_cell_freq_);
        }

        ALWAYS_INLINE ObjectsFilter
        GetFilteredSubmatrixFor(int x1, int y1, int x2, int y2) const noexcept {
            VERIFY(are_submatrices_calculated_);
            return prefix_submatrix_[x2 * height_ + y2] &
                   suffix_submatrix_[x1 * height_ + y1];
        }

        geometry::Box objects_bounding_box_ = geometry::Box::Endless();
        double grid_cell_freq_ = 1.0;
        int height_ = 0;
        int width_ = 0;

        std::vector<ObjectsFilter> slice_objects_sets_;
        std::vector<ObjectsFilter> prefix_submatrix_;
        std::vector<ObjectsFilter> suffix_submatrix_;
        bool are_submatrices_calculated_ = false;
        ObjectsFilter all_slice_objects_{};
    };

    template <typename GeometryType, typename ObjectsFilter = geometry::BitsetWrapper<1024>>
    class StaticObjectsGrid {
    public:
        using Scalar = ScalarOf<GeometryType>;
        using Slice = geometry::SliceT<ObjectsFilter>;

        explicit StaticObjectsGrid(double grid_cell_size)
            : grid_cell_size_(grid_cell_size)
            , grid_cell_freq_(1.0 / grid_cell_size)
        {
        }

        StaticObjectsGrid(const StaticObjectsGrid&) noexcept = delete;
        StaticObjectsGrid(StaticObjectsGrid&&) noexcept = default;

        void SetObjects(std::vector<GeometryType> objects) noexcept {
            objects_ = std::move(objects);

            // Calculate bounding boxes for all objects
            geometry::Box grid_objects_bounding_box(geometry::Box::Endless());
            for (size_t ind = 0; ind < objects_.size(); ++ind) {
                const auto& object = objects_[ind];
                grid_objects_bounding_box.expand(object);
            }

            grid_ = Slice(grid_objects_bounding_box, grid_cell_freq_);

            for (size_t ind = 0; ind < std::size(objects_); ++ind) {
                const auto& object = objects_[ind];
                grid_.AddObjectToSlice(ind, object);
            }

            // Precalculate statistics for grid submatrices in order to speed up queries
            grid_.CalculateFilterSubmatrices();
        }

        template <typename RequestGeometryType, typename ObjectFilterFunc>
        ALWAYS_INLINE void ForEachObjectNearGeometry(
            const RequestGeometryType& geometry,
            ObjectFilterFunc object_filter,
            ObjectsFilter objects_to_consider,
            Scalar request_radius) const noexcept {
            objects_to_consider &= grid_.GetObjectsMaskForGeometry(geometry, request_radius);
            objects_to_consider.for_each(
                [this, &geometry, &object_filter](const size_t current_object_idx) {
                    object_filter(geometry, objects_[current_object_idx], current_object_idx);
                });
        }

        template <typename RequestGeometryType, typename ObjectFilterFunc>
        ALWAYS_INLINE void ForEachObjectNearGeometry(
            const RequestGeometryType& geometry,
            ObjectFilterFunc object_filter,
            Scalar request_radius) const noexcept {
            grid_.GetObjectsMaskForGeometry(geometry, request_radius)
                .for_each([this, &geometry, &object_filter](const size_t current_object_idx) {
                    object_filter(geometry, objects_[current_object_idx], current_object_idx);
                });
        }

        template <typename RequestGeometryType>
        auto GetClosestObjectDistanceSq(
            const RequestGeometryType& geometry,
            ObjectsFilter objects_to_consider,
            Scalar request_radius) const noexcept {
            auto result = request_radius * request_radius;
            ForEachObjectNearGeometry(
                geometry,
                [&result](const auto& geometry, const auto& object, const auto /*current_object_idx*/) {
                    result = std::min(result, geometry::DistanceSq(geometry, object));
                },
                objects_to_consider,
                request_radius);
            return result;
        }

    private:
        Slice grid_;
        std::vector<GeometryType> objects_;
        double grid_cell_size_ = 1.0;
        double grid_cell_freq_ = 1.0;
    };

} // namespace sdc::geometry
