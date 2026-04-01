#pragma once

#include "geometry/angle.h"
#include "geometry/box.h"
#include "geometry/convert.h"
#include "geometry/directed_point.h"
#include "geometry/distance.h"
#include "geometry/lazy_transformation_geometry.h"
#include "geometry/vector.h"
#include "geometry_utils/static_objects_grid/static_objects_grid.h"
#include "kinematics/concepts/basic/basic.h"
#include "kinematics/convert.h"
#include "kinematics/position.h"
#include <math/concepts/basic.h>
#include <math/math.h>
#include <optional>
#include <time/checks.h>
#include <time/concepts/basic.h>
#include <time/concepts/properties/properties.h>
#include <time/int_clock.h>
#include "common/macros.h"
#include <tools/assert_stream.h>
#include <tools/parallel_algorithm.h>
#include <tools/smart_ptr.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "common/macros.h"
#include "geometry_utils/objects_grid/bitset_wrapper.h"

namespace sdc::kinematics {

    namespace impl {

        template <typename T>
        concept IsDereferencable = requires(T a) {
            { *a };
        };

        template <typename T>
        concept IsReferenceWrapper = requires(T a) {
            { a.get() } -> std::same_as<const typename T::type&>;
        };

    } // namespace impl

    enum class GeometryAccessType {
        kFast,
        kAccurate,
        kOccupancyGrid,
    };

    template <typename GeometryType, time::IsTimestamp TimeT = std::chrono::time_point<time::IntClock>>
    class DynamicObject {
    public:
        virtual ~DynamicObject() = default;

        using OriginalGeometry = GeometryType;
        using DynamicGeometry = geometry::LazyTransformationGeometry<GeometryType>;
        using Time = TimeT;
        using Scalar = geometry::ScalarOf<GeometryType>;
        using Point = geometry::PointT<Scalar>;
        using Vector = geometry::VectorT<Scalar>;
        using Direction = geometry::DirectionT<Scalar>;
        using DirectedPoint = geometry::DirectedPointT<Point>;

        virtual bool IsObjectExistsAt(Time /*timestamp*/) const noexcept {
            return true;
        }
        virtual bool IsStationary() const noexcept {
            return false;
        }
        virtual DynamicGeometry GetFastGeometryAtTime(Time /*timestamp*/) const noexcept = 0;
        virtual DynamicGeometry GetAccurateGeometryAtTime(Time /*timestamp*/) const noexcept = 0;
        virtual DynamicGeometry GetGridGeometryAtTime(Time /*timestamp*/) const noexcept = 0;

        virtual kinematics::TemporalPosition GetPositionAtTime(Time /*timestamp*/) const noexcept = 0;
    };

    template <typename GeometryType, time::IsTimestamp TimeT = std::chrono::time_point<time::IntClock>>
    class RegularStationaryDynamicObject final: public DynamicObject<GeometryType, TimeT> {
    public:
        using Base = DynamicObject<GeometryType, TimeT>;
        using typename Base::Direction;
        using typename Base::DynamicGeometry;
        using typename Base::OriginalGeometry;
        using typename Base::Point;
        using typename Base::Scalar;
        using typename Base::Time;
        using typename Base::Vector;

        explicit RegularStationaryDynamicObject(GeometryType geometry)
            : geometry_(std::move(geometry))
        {
        }

        bool IsStationary() const noexcept override {
            return true;
        }

        DynamicGeometry GetFastGeometryAtTime(Time /*timestamp*/) const noexcept override {
            return DynamicGeometry(
                geometry_, Vector::Zero(), Vector::Zero(), Direction::Zero());
        }
        DynamicGeometry GetAccurateGeometryAtTime(Time /*timestamp*/) const noexcept override {
            return DynamicGeometry(
                geometry_, Vector::Zero(), Vector::Zero(), Direction::Zero());
        }
        DynamicGeometry GetGridGeometryAtTime(Time /*timestamp*/) const noexcept override {
            return DynamicGeometry(
                geometry_, Vector::Zero(), Vector::Zero(), Direction::Zero());
        }
        TemporalPosition GetPositionAtTime(Time timestamp) const noexcept override {
            TemporalPosition result{};
            // result.p = geometry::Centroid(geometry_);
            result.time = time::Convert<ros::Time>(timestamp);
            return result;
        }

        const OriginalGeometry& GetOriginalGeometry() const noexcept {
            return geometry_;
        }

    private:
        GeometryType geometry_;
    };

    template <typename GeometryType, time::IsTimestamp TimeT = std::chrono::time_point<time::IntClock>>
    class StationaryDynamicObject final: public DynamicObject<GeometryType, TimeT> {
    public:
        using Base = DynamicObject<GeometryType, TimeT>;
        using typename Base::Direction;
        using typename Base::DynamicGeometry;
        using typename Base::OriginalGeometry;
        using typename Base::Point;
        using typename Base::Scalar;
        using typename Base::Time;
        using typename Base::Vector;

        explicit StationaryDynamicObject(GeometryType fast_geometry)
            : fast_geometry_(std::move(fast_geometry))
        {
        }
        explicit StationaryDynamicObject(GeometryType fast_geometry, GeometryType accurate_geometry)
            : fast_geometry_(std::move(fast_geometry))
            , accurate_geometry_(std::move(accurate_geometry))
        {
        }
        explicit StationaryDynamicObject(
            GeometryType fast_geometry, GeometryType accurate_geometry, GeometryType grid_geometry)
            : fast_geometry_(std::move(fast_geometry))
            , accurate_geometry_(std::move(accurate_geometry))
            , grid_geometry_(std::move(grid_geometry))
        {
        }

        bool IsStationary() const noexcept override {
            return true;
        }

        DynamicGeometry GetFastGeometryAtTime(Time /*timestamp*/) const noexcept override {
            return DynamicGeometry(
                fast_geometry_, Vector::Zero(), Vector::Zero(), Direction::Zero());
        }
        DynamicGeometry GetAccurateGeometryAtTime(Time timestamp) const noexcept override {
            if (accurate_geometry_) {
                return DynamicGeometry(
                    *accurate_geometry_, Vector::Zero(), Vector::Zero(), Direction::Zero());
            } else {
                return GetFastGeometryAtTime(timestamp);
            }
        }

        DynamicGeometry GetGridGeometryAtTime(Time timestamp) const noexcept override {
            if (grid_geometry_) {
                return DynamicGeometry(
                    *grid_geometry_, Vector::Zero(), Vector::Zero(), Direction::Zero());
            } else {
                return GetAccurateGeometryAtTime(timestamp);
            }
        }

        TemporalPosition GetPositionAtTime(Time timestamp) const noexcept override {
            TemporalPosition result;
            // result.p = geometry::Centroid(fast_geometry_);
            result.time = time::Convert<ros::Time>(timestamp);
            return result;
        }

        const OriginalGeometry& GetOriginalFastGeometry() const noexcept {
            return fast_geometry_;
        }
        const OriginalGeometry& GetOriginalAccurateGeometry() const noexcept {
            return *accurate_geometry_;
        }
        const OriginalGeometry& GetOriginalGridGeometry() const noexcept {
            return *grid_geometry_;
        }

    private:
        GeometryType fast_geometry_;
        std::optional<GeometryType> accurate_geometry_;
        std::optional<GeometryType> grid_geometry_;
    };

    template <
        typename GeometryType,
        typename MotionSplineT,
        time::IsTimestamp TimeT = std::chrono::time_point<time::IntClock>>
    class MotionSplineDynamicObject final: public DynamicObject<GeometryType, TimeT> {
    public:
        using Base = DynamicObject<GeometryType, TimeT>;
        using typename Base::Direction;
        using typename Base::DynamicGeometry;
        using typename Base::OriginalGeometry;
        using typename Base::Point;
        using typename Base::Scalar;
        using typename Base::Time;
        using typename Base::Vector;

        explicit MotionSplineDynamicObject(
            MotionSplineT point,
            GeometryType fast_geometry,
            Point fast_geometry_base,
            std::optional<TimeT> max_timestamp = std::nullopt,
            bool prolong_object_lifetime = false)
            : motion_spline_(std::move(point))
            , fast_geometry_(std::move(fast_geometry))
            , fast_geometry_base_(geometry::Convert<Vector>(fast_geometry_base))
            , max_timestamp_(max_timestamp)
            , prolong_object_lifetime_(prolong_object_lifetime)
        {
        }

        explicit MotionSplineDynamicObject(
            MotionSplineT point,
            GeometryType fast_geometry,
            Point fast_geometry_base,
            GeometryType accurate_geometry,
            Point accurate_geometry_base,
            std::optional<TimeT> max_timestamp = std::nullopt,
            bool prolong_object_lifetime = false)
            : motion_spline_(std::move(point))
            , fast_geometry_(geometry::Convert<Vector>(fast_geometry))
            , fast_geometry_base_(std::move(fast_geometry_base))
            , max_timestamp_(max_timestamp)
            , accurate_geometry_(std::move(accurate_geometry))
            , accurate_geometry_base_(geometry::Convert<Vector>(accurate_geometry_base))
            , prolong_object_lifetime_(prolong_object_lifetime)
        {
        }

        explicit MotionSplineDynamicObject(
            MotionSplineT point,
            GeometryType fast_geometry,
            Point fast_geometry_base,
            GeometryType accurate_geometry,
            Point accurate_geometry_base,
            GeometryType grid_geometry,
            Point grid_geometry_base,
            std::optional<TimeT> max_timestamp = std::nullopt,
            bool prolong_object_lifetime = false)
            : motion_spline_(std::move(point))
            , fast_geometry_(std::move(fast_geometry))
            , fast_geometry_base_(geometry::Convert<Vector>(fast_geometry_base))
            , max_timestamp_(max_timestamp)
            , accurate_geometry_(std::move(accurate_geometry))
            , accurate_geometry_base_(geometry::Convert<Vector>(accurate_geometry_base))
            , grid_geometry_(std::move(grid_geometry))
            , grid_geometry_base_(geometry::Convert<Vector>(grid_geometry_base))
            , prolong_object_lifetime_(prolong_object_lifetime)
        {
        }

        bool IsObjectExistsAt(Time timestamp) const noexcept override {
            return prolong_object_lifetime_ || !max_timestamp_ || timestamp <= *max_timestamp_;
        }

        DynamicGeometry GetFastGeometryAtTime(Time timestamp) const noexcept override {
            const auto effective_timestamp = GetEffectiveTimestamp(timestamp);
            const auto moved_point = kinematics::MoveTo(motion_spline_, effective_timestamp);
            return DynamicGeometry(
                fast_geometry_,
                fast_geometry_base_,
                geometry::Convert<Vector>(kinematics::GetPosition(moved_point)) - geometry::Convert<Vector>(kinematics::GetPosition(motion_spline_)),
                geometry::Direction::FromDir(kinematics::GetDirection(moved_point)) -
                    geometry::Direction::FromDir(kinematics::GetDirection(motion_spline_)));
        }
        DynamicGeometry GetAccurateGeometryAtTime(Time timestamp) const noexcept override {
            const auto effective_timestamp = GetEffectiveTimestamp(timestamp);
            if (accurate_geometry_) {
                const auto moved_point = kinematics::MoveTo(motion_spline_, effective_timestamp);
                return DynamicGeometry(
                    *accurate_geometry_,
                    *accurate_geometry_base_,
                    geometry::Convert<Vector>(kinematics::GetPosition(moved_point)) - geometry::Convert<Vector>(kinematics::GetPosition(motion_spline_)),
                    geometry::Direction::FromDir(kinematics::GetDirection(moved_point)) -
                        geometry::Direction::FromDir(kinematics::GetDirection(motion_spline_)));
            } else {
                return GetFastGeometryAtTime(effective_timestamp);
            }
        }

        DynamicGeometry GetGridGeometryAtTime(Time timestamp) const noexcept override {
            const auto effective_timestamp = GetEffectiveTimestamp(timestamp);
            if (grid_geometry_) {
                const auto moved_point = kinematics::MoveTo(motion_spline_, effective_timestamp);
                return DynamicGeometry(
                    *grid_geometry_,
                    *grid_geometry_base_,
                    geometry::Convert<Vector>(kinematics::GetPosition(moved_point)) - geometry::Convert<Vector>(kinematics::GetPosition(motion_spline_)),
                    geometry::Direction::FromDir(kinematics::GetDirection(moved_point)) -
                        geometry::Direction::FromDir(kinematics::GetDirection(motion_spline_)));
            } else {
                return GetAccurateGeometryAtTime(effective_timestamp);
            }
        }

        TemporalPosition GetPositionAtTime(Time timestamp) const noexcept override {
            const auto effective_timestamp = GetEffectiveTimestamp(timestamp);
            const auto moved_point = kinematics::MoveTo(motion_spline_, effective_timestamp);
            return kinematics::Convert<TemporalPosition>(moved_point);
        }

        const OriginalGeometry& GetOriginalFastGeometry() const noexcept {
            return fast_geometry_;
        }
        const OriginalGeometry& GetOriginalAccurateGeometry() const noexcept {
            return *accurate_geometry_;
        }
        const OriginalGeometry& GetOriginalGridGeometry() const noexcept {
            return *grid_geometry_;
        }

        const auto& GetMotionSpline() const noexcept {
            return motion_spline_;
        }

    private:
        auto GetEffectiveTimestamp(Time timestamp) const noexcept {
            if (prolong_object_lifetime_ && max_timestamp_.has_value()) {
                return std::min(timestamp, *max_timestamp_);
            }
            return timestamp;
        }

        MotionSplineT motion_spline_;
        GeometryType fast_geometry_;
        Vector fast_geometry_base_;
        std::optional<TimeT> max_timestamp_;
        std::optional<GeometryType> accurate_geometry_;
        std::optional<Vector> accurate_geometry_base_;
        std::optional<GeometryType> grid_geometry_;
        std::optional<Vector> grid_geometry_base_;
        bool prolong_object_lifetime_ = false;
    };

    template <
        typename GeometryType,
        typename TrajectoryT,
        time::IsTimestamp TimeT = std::chrono::time_point<time::IntClock>>
        requires(kinematics::IsTemporalTrajectory<TrajectoryT>)
    class TrajectoryDynamicObject final: public DynamicObject<GeometryType, TimeT> {
    public:
        using Base = DynamicObject<GeometryType, TimeT>;
        using typename Base::DirectedPoint;
        using typename Base::Direction;
        using typename Base::DynamicGeometry;
        using typename Base::OriginalGeometry;
        using typename Base::Point;
        using typename Base::Scalar;
        using typename Base::Time;
        using typename Base::Vector;

        explicit TrajectoryDynamicObject(
            TrajectoryT trajectory,
            GeometryType fast_geometry,
            DirectedPoint fast_geometry_base,
            std::optional<TimeT> max_timestamp = std::nullopt,
            bool prolong_object_lifetime = false)
            : trajectory_(std::move(trajectory))
            , fast_geometry_(std::move(fast_geometry))
            , fast_geometry_base_(fast_geometry_base)
            , max_timestamp_(max_timestamp)
            , prolong_object_lifetime_(prolong_object_lifetime)
        {
            BuildTimestamps();
        }

        explicit TrajectoryDynamicObject(
            TrajectoryT trajectory,
            GeometryType fast_geometry,
            DirectedPoint fast_geometry_base,
            GeometryType accurate_geometry,
            DirectedPoint accurate_geometry_base,
            std::optional<TimeT> max_timestamp = std::nullopt,
            bool prolong_object_lifetime = false)
            : trajectory_(std::move(trajectory))
            , fast_geometry_(fast_geometry)
            , fast_geometry_base_(std::move(fast_geometry_base))
            , max_timestamp_(max_timestamp)
            , accurate_geometry_(std::move(accurate_geometry))
            , accurate_geometry_base_(accurate_geometry_base)
            , prolong_object_lifetime_(prolong_object_lifetime)
        {
            BuildTimestamps();
        }

        explicit TrajectoryDynamicObject(
            TrajectoryT trajectory,
            GeometryType fast_geometry,
            DirectedPoint fast_geometry_base,
            GeometryType accurate_geometry,
            DirectedPoint accurate_geometry_base,
            GeometryType grid_geometry,
            DirectedPoint grid_geometry_base,
            std::optional<TimeT> max_timestamp = std::nullopt,
            bool prolong_object_lifetime = false)
            : trajectory_(std::move(trajectory))
            , fast_geometry_(std::move(fast_geometry))
            , fast_geometry_base_(fast_geometry_base)
            , max_timestamp_(max_timestamp)
            , accurate_geometry_(std::move(accurate_geometry))
            , accurate_geometry_base_(accurate_geometry_base)
            , grid_geometry_(std::move(grid_geometry))
            , grid_geometry_base_(grid_geometry_base)
            , prolong_object_lifetime_(prolong_object_lifetime)
        {
            BuildTimestamps();
        }

        bool IsObjectExistsAt(Time timestamp) const noexcept override {
            return prolong_object_lifetime_ || !max_timestamp_ || timestamp <= *max_timestamp_;
        }

        kinematics::TemporalPosition GetPositionAtTime(Time timestamp) const noexcept override {
            const auto effective_timestamp = GetEffectiveTimestamp(timestamp);
            const auto& cont = kinematics::GetContainer(trajectory_);
            ptrdiff_t next_index = std::distance(
                timestamps_.begin(), std::lower_bound(timestamps_.begin(), timestamps_.end(), effective_timestamp));
            next_index = math::clamp<ptrdiff_t>(next_index, 1, cont.size() - 1);
            const auto& a = cont[next_index - 1];
            const auto& b = cont[next_index];
            const double delta = time::GetTimestampAsDoubleSeconds(b) - time::GetTimestampAsDoubleSeconds(a);
            const double ratio =
                delta > 0.0 ? (time::GetTimestampAsDoubleSeconds(effective_timestamp) - time::GetTimestampAsDoubleSeconds(a)) / delta : 0.0;

            return kinematics::TemporalPosition{
                .p = geometry::Extrapolate(kinematics::GetPosition(a), kinematics::GetPosition(b), ratio),
                .yaw = geometry::GetYaw(kinematics::approx::DirectionBetween(a, b)),
                .curvature = math::lerp(kinematics::GetCurvature(a), kinematics::GetCurvature(b), ratio),
                .time = time::Convert<ros::Time>(effective_timestamp),
                .speed = math::lerp(kinematics::GetSpeed(a), kinematics::GetSpeed(b), ratio),
                .tangent_acceleration = math::lerp(kinematics::GetTangentAcceleration(a), kinematics::GetTangentAcceleration(b), ratio),
            };
        }

        DynamicGeometry GetFastGeometryAtTime(Time timestamp) const noexcept override {
            const auto effective_timestamp = GetEffectiveTimestamp(timestamp);
            const auto pos = GetPositionAtTime(effective_timestamp);
            return DynamicGeometry(
                fast_geometry_,
                geometry::Convert<Vector>(fast_geometry_base_),
                geometry::Convert<Vector>(kinematics::GetPosition(pos)) - geometry::Convert<Vector>(fast_geometry_base_),
                geometry::Direction::FromDir(kinematics::GetDirection(pos)) -
                    geometry::Direction{geometry::GetUnitDir(fast_geometry_base_)});
        }
        DynamicGeometry GetAccurateGeometryAtTime(Time timestamp) const noexcept override {
            const auto effective_timestamp = GetEffectiveTimestamp(timestamp);
            if (accurate_geometry_) {
                const auto pos = GetPositionAtTime(effective_timestamp);
                return DynamicGeometry(
                    *accurate_geometry_,
                    geometry::Convert<Vector>(*accurate_geometry_base_),
                    geometry::Convert<Vector>(kinematics::GetPosition(pos)) - geometry::Convert<Vector>(accurate_geometry_base_),
                    geometry::Direction::FromDir(kinematics::GetDirection(pos)) -
                        geometry::Direction{geometry::GetUnitDir(accurate_geometry_base_)});
            } else {
                return GetFastGeometryAtTime(effective_timestamp);
            }
        }

        DynamicGeometry GetGridGeometryAtTime(Time timestamp) const noexcept override {
            const auto effective_timestamp = GetEffectiveTimestamp(timestamp);
            if (grid_geometry_) {
                const auto pos = GetPositionAtTime(effective_timestamp);
                return DynamicGeometry(
                    *grid_geometry_,
                    geometry::Convert<Vector>(*grid_geometry_base_),
                    geometry::Convert<Vector>(kinematics::GetPosition(pos)) - geometry::Convert<Vector>(grid_geometry_base_),
                    geometry::Direction::FromDir(kinematics::GetDirection(pos)) -
                        geometry::Direction{geometry::GetUnitDir(grid_geometry_base_)});
            } else {
                return GetAccurateGeometryAtTime(effective_timestamp);
            }
        }

        const OriginalGeometry& GetOriginalFastGeometry() const noexcept {
            return fast_geometry_;
        }
        const OriginalGeometry& GetOriginalAccurateGeometry() const noexcept {
            return *accurate_geometry_;
        }
        const OriginalGeometry& GetOriginalGridGeometry() const noexcept {
            return *grid_geometry_;
        }

        const auto& GetTrajectory() const noexcept {
            return trajectory_;
        }

    private:
        void BuildTimestamps() noexcept {
            VERIFY(time::IsTimeMonotonic(trajectory_));
            const auto& cont = kinematics::GetContainer(trajectory_);
            VERIFY(cont.size() >= 2);
            for (const auto& pose : cont) {
                timestamps_.push_back(time::Convert<TimeT>(time::GetTimestamp(pose)));
            }
        }

        auto GetEffectiveTimestamp(Time timestamp) const noexcept {
            if (prolong_object_lifetime_ && max_timestamp_.has_value()) {
                return std::min(timestamp, *max_timestamp_);
            }
            return timestamp;
        }

        TrajectoryT trajectory_;
        std::vector<TimeT> timestamps_;
        GeometryType fast_geometry_;
        DirectedPoint fast_geometry_base_;
        std::optional<TimeT> max_timestamp_;
        std::optional<GeometryType> accurate_geometry_;
        std::optional<DirectedPoint> accurate_geometry_base_;
        std::optional<GeometryType> grid_geometry_;
        std::optional<DirectedPoint> grid_geometry_base_;
        bool prolong_object_lifetime_ = false;
    };

    template <
        typename DynamicObjectT,
        typename ObjectsFilterT = geometry::BitsetWrapper<1024>,
        time::IsTimestamp TimeT = std::chrono::time_point<time::IntClock>>
    class DynamicObjectsGrid {
    public:
        using ObjectsFilter = ObjectsFilterT;
        using Slice = geometry::SliceT<ObjectsFilter>;
        using Time = TimeT;
        using Duration = typename TimeT::duration;
        using DynamicObject = tools::remove_smart_ptr_t<DynamicObjectT>;
        using OriginalGeometry = typename DynamicObject::OriginalGeometry;
        using DynamicGeometry = typename DynamicObject::DynamicGeometry;
        using Scalar = geometry::ScalarOf<OriginalGeometry>;

        DynamicObjectsGrid(
            double grid_cell_size, Time start_timestamp, size_t slices_count, Duration time_step) noexcept
            : grid_cell_size_(grid_cell_size)
            , grid_cell_freq_(math::one<Scalar>() / grid_cell_size_)
            , start_timestamp_(start_timestamp)
            , time_step_(time_step)
        {
            slices_.resize(slices_count);
        }
        DynamicObjectsGrid(const DynamicObjectsGrid&) noexcept = delete;
        DynamicObjectsGrid(DynamicObjectsGrid&&) noexcept = default;

        template <GeometryAccessType access_type = GeometryAccessType::kFast>
        void SetObjects(std::vector<DynamicObjectT> objects) {
            objects_ = std::move(objects);
            VERIFY_STREAM(
                objects_.size() <= ObjectsFilter::max_stored_objects_count,
                "Too many objects: " << objects_.size() << " > "
                                     << ObjectsFilter::max_stored_objects_count);
            for (size_t slice_idx = 0; slice_idx < slices_.size(); ++slice_idx) {
                FillSlice<access_type>(slice_idx);
            }
        }

        template <GeometryAccessType access_type = GeometryAccessType::kFast, typename ThreadPool>
        void SetObjects(std::vector<DynamicObjectT> objects, std::shared_ptr<ThreadPool> thread_pool) {
            objects_ = std::move(objects);
            VERIFY_STREAM(
                objects_.size() <= ObjectsFilter::max_stored_objects_count,
                "Too many objects: " << objects_.size() << " > "
                                     << ObjectsFilter::max_stored_objects_count);
            parallel::For(slices_.size(), [this](size_t slice_idx) {
                FillSlice<access_type>(slice_idx);
            }, thread_pool.get(), parallel::TimeLoggerName("ThreadedPart"));
        }

        const auto& GetObjects() const noexcept {
            return objects_;
        }

        std::vector<std::reference_wrapper<const DynamicObjectT>> GetObjectsWithFilter(ObjectsFilter filter) const noexcept {
            std::vector<std::reference_wrapper<const DynamicObjectT>> result;
            filter.for_each(
                [this, &result](const size_t current_object_idx) {
                    result.push_back(std::cref(objects_[current_object_idx]));
                });

            return result;
        }

        template <typename RequestGeometryType, typename ObjectFilterFunc>
        ALWAYS_INLINE void ForEachObjectNearGeometry(
            const Time timestamp,
            const RequestGeometryType& geometry,
            ObjectFilterFunc object_filter,
            ObjectsFilter objects_to_consider,
            Scalar request_radius) const noexcept {
            objects_to_consider &= GetObjectsMaskForGeometryAt(timestamp, geometry, request_radius);
            objects_to_consider.for_each(
                [this, &timestamp, &geometry, &object_filter](const size_t current_object_idx) {
                    if (GetObjectRef(objects_[current_object_idx]).IsObjectExistsAt(timestamp)) {
                        object_filter(
                            geometry, GetObjectRef(objects_[current_object_idx]), current_object_idx);
                    }
                });
        }

        template <typename RequestGeometryType, typename ObjectFilterFunc>
        ALWAYS_INLINE void ForEachObjectNearGeometry(
            const Time timestamp,
            const RequestGeometryType& geometry,
            ObjectFilterFunc object_filter,
            Scalar request_radius) const noexcept {
            GetObjectsMaskForGeometryAt(timestamp, geometry, request_radius)
                .for_each([this, &timestamp, &geometry, &object_filter](const size_t current_object_idx) {
                    if (GetObjectRef(objects_[current_object_idx]).IsObjectExistsAt(timestamp)) {
                        object_filter(
                            geometry, GetObjectRef(objects_[current_object_idx]), current_object_idx);
                    }
                });
        }

        template <GeometryAccessType access_type, typename RequestGeometryType>
        auto GetClosestObjectDistanceSq(
            const Time timestamp,
            const RequestGeometryType& geometry,
            ObjectsFilter objects_to_consider,
            Scalar request_radius) const noexcept {
            auto result = math::squared(request_radius);
            ForEachObjectNearGeometry(
                timestamp,
                geometry,
                [&result, timestamp](
                    const auto& geometry, const auto& object, const auto /*current_object_idx*/) {
                    math::UpdateMin(
                        result,
                        geometry::DistanceSq(geometry, GetObjectGeometryAt<access_type>(object, timestamp)));
                },
                objects_to_consider,
                request_radius);
            return result;
        }

        template <GeometryAccessType access_type, typename RequestGeometryType>
        auto GetClosestObjectDistanceSq(
            const Time timestamp,
            const RequestGeometryType& geometry,
            Scalar request_radius) const noexcept {
            auto result = math::squared(request_radius);
            ForEachObjectNearGeometry(
                timestamp,
                geometry,
                [&result, timestamp](
                    const auto& geometry, const auto& object, const auto /*current_object_idx*/) {
                    math::UpdateMin(
                        result,
                        geometry::DistanceSq(geometry, GetObjectGeometryAt<access_type>(object, timestamp)));
                },
                request_radius);
            return result;
        }

        TimeT GetMinAllowedGridTimestamp() const noexcept {
            return start_timestamp_ - time_step_ / 2;
        }
        TimeT GetMinTimestamp() const noexcept {
            return start_timestamp_;
        }
        TimeT GetMaxAllowedGridTimestamp() const noexcept {
            return start_timestamp_ + time_step_ * slices_.size() + time_step_ / 2;
        }
        TimeT GetMaxTimestamp() const noexcept {
            return start_timestamp_ + time_step_ * slices_.size();
        }
        Duration GetTimeStep() const noexcept {
            return time_step_;
        }

        template <typename RequestGeometryType>
        ALWAYS_INLINE ObjectsFilter GetObjectsMaskForGeometryAt(
            Time timestamp, const RequestGeometryType& geometry, double request_radius) const noexcept {
            VERIFY_STREAM(
                timestamp >= start_timestamp_ - time_step_ &&
                    timestamp < start_timestamp_ + time_step_ * slices_.size() + time_step_,
                "Request out of DynamicObjectsGrid time range: tm="
                    << std::fixed << std::setprecision(15)
                    << time::GetTimestampAsDoubleSeconds(timestamp)
                    << " start_timestamp=" << time::GetTimestampAsDoubleSeconds(start_timestamp_)
                    << " time_step=" << time::GetDurationAsDoubleSeconds(time_step_)
                    << " slices.size()=" << slices_.size()
                    << " end_timestamp="
                    << time::GetTimestampAsDoubleSeconds(TimeT(start_timestamp_ + time_step_ * slices_.size())));
            return slices_[GetGridIndex(timestamp)].GetObjectsMaskForGeometry(geometry, request_radius);
        }

        static const DynamicObject& GetObjectRef(const DynamicObjectT& object) noexcept {
            if constexpr (impl::IsDereferencable<DynamicObjectT>) {
                return *object;
            } else if constexpr (impl::IsReferenceWrapper<DynamicObjectT>) {
                return object.get();
            } else {
                return object;
            }
        }

        template <GeometryAccessType access_type>
        static decltype(auto) GetObjectGeometryAt(
            const DynamicObject& object, Time timestamp) noexcept {
            if constexpr (access_type == GeometryAccessType::kFast) {
                return object.GetFastGeometryAtTime(timestamp);
            } else if constexpr (access_type == GeometryAccessType::kAccurate) {
                return object.GetAccurateGeometryAtTime(timestamp);
            } else {
                return object.GetGridGeometryAtTime(timestamp);
            }
        }

        template <GeometryAccessType access_type>
        static decltype(auto) GetObjectGeometryAt(
            const DynamicObjectT& object, Time timestamp) noexcept
            requires(!std::same_as<DynamicObjectT, DynamicObject>)
        {
            if constexpr (access_type == GeometryAccessType::kFast) {
                return GetObjectRef(object).GetFastGeometryAtTime(timestamp);
            } else if constexpr (access_type == GeometryAccessType::kAccurate) {
                return GetObjectRef(object).GetAccurateGeometryAtTime(timestamp);
            } else {
                return GetObjectRef(object).GetGridGeometryAtTime(timestamp);
            }
        }

    private:
        ALWAYS_INLINE size_t GetGridIndex(Time timestamp) const noexcept {
            timestamp = std::max(timestamp, start_timestamp_);
            size_t grid_index = static_cast<size_t>((timestamp - start_timestamp_) / time_step_);
            return std::min(grid_index, slices_.size() - 1);
        }

        template <GeometryAccessType access_type>
        void FillSlice(size_t slice_idx) {
            auto& slice = slices_[slice_idx];
            const Time slice_tm = start_timestamp_ + time_step_ * slice_idx;
            geometry::Box envelope = geometry::Box::Endless();
            for (const auto& object : objects_) {
                if (GetObjectRef(object).IsObjectExistsAt(slice_tm)) {
                    envelope.expand(GetObjectGeometryAt<access_type>(object, slice_tm));
                }
                if (GetObjectRef(object).IsObjectExistsAt(slice_tm + time_step_ / 2)) {
                    envelope.expand(GetObjectGeometryAt<access_type>(object, slice_tm + time_step_ / 2));
                }
                if (GetObjectRef(object).IsObjectExistsAt(slice_tm - time_step_ / 2)) {
                    envelope.expand(GetObjectGeometryAt<access_type>(object, slice_tm - time_step_ / 2));
                }
            }

            slice = Slice(envelope, grid_cell_freq_);
            for (size_t object_idx = 0; object_idx < objects_.size(); ++object_idx) {
                const auto& object = objects_[object_idx];
                if (GetObjectRef(object).IsObjectExistsAt(slice_tm)) {
                    slice.AddObjectToSlice(
                        object_idx, GetObjectGeometryAt<access_type>(object, slice_tm));
                }
                if (GetObjectRef(object).IsObjectExistsAt(slice_tm + time_step_ / 2)) {
                    slice.AddObjectToSlice(
                        object_idx,
                        GetObjectGeometryAt<access_type>(object, slice_tm + time_step_ / 2));
                }
                if (GetObjectRef(object).IsObjectExistsAt(slice_tm - time_step_ / 2)) {
                    slice.AddObjectToSlice(
                        object_idx,
                        GetObjectGeometryAt<access_type>(object, slice_tm - time_step_ / 2));
                }
            }
            slice.CalculateFilterSubmatrices();
        }

        Scalar grid_cell_size_;
        Scalar grid_cell_freq_;
        Time start_timestamp_;
        Duration time_step_;
        std::vector<Slice> slices_;
        std::vector<DynamicObjectT> objects_;
    };

    template <typename GeometryType, time::IsTimestamp TimeT = std::chrono::time_point<time::IntClock>>
    kinematics::TemporalPositions SampleDynamicObjectTrajectory(
        const DynamicObject<GeometryType, TimeT>& dynamic_object,
        const ros::Time start_time,
        const ros::Time end_time,
        const ros::Duration duration,
        const double trajectory_min_step_length,
        const double min_candidate_distance) {
        kinematics::TemporalPositions result;

        for (ros::Time current_time = start_time; current_time <= end_time; current_time += duration) {
            auto time_tm = time::Convert<TimeT>(current_time);
            if (dynamic_object.IsObjectExistsAt(time_tm)) {
                result.emplace_back(dynamic_object.GetPositionAtTime(time_tm));
            }
        }

        ThrowOutClosePoints(result, trajectory_min_step_length, min_candidate_distance);
        return result;
    }

} // namespace sdc::kinematics
