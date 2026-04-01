#include <gtest/gtest.h>
#include <ros/time.h>
#include <range/v3/all.hpp>

#include "ros/duration.h"
#include "geometry/angle.h"
#include "geometry/baton.h"
#include "geometry/centroid.h"
#include "geometry/circle.h"
#include "geometry/convert.h"
#include "geometry/distance.h"
#include "geometry/envelope.h"
#include "geometry/lazy_transform_geometry.h"
#include "geometry/length.h"
#include "geometry/normalized_box.h"
#include "geometry/point.h"
#include "geometry/random.h"
#include "geometry/rectangle.h"
#include "geometry/rectangle_precalc.h"
#include "geometry/rotation.h"
#include "geometry/scale.h"
#include "geometry/segment.h"
#include "geometry/translation.h"
#include "geometry/unit_vector.h"
#include "geometry/vector.h"
#include <geometry_utils/precalc.h>
#include "kinematics/concepts/properties/get_properties.h"
#include "kinematics/convert.h"
#include "kinematics/dynamic_objects_grid/dynamic_objects_grid.h"
#include "kinematics/motion_n_spline.h"
#include "kinematics/type_of_direction.h"
#include <math/concepts/basic.h>
#include <math/math.h>
#include <math/random.h>
#include <simd/concepts.h>
#include <time/concepts/properties/properties.h>
#include <time/int_clock.h>
#include <time/random.h>
#include "common/macros.h"

namespace views = ranges::views;

namespace sdc::kinematics {

    TEST(DynamicObjectsGrid, Stationary) {
        std::mt19937 rd(42);

        for (size_t iteration = 0; iteration < 200; ++iteration) {
            const auto start_tm = time::IntClock::now();
            const auto step_tm = std::chrono::milliseconds(100);
            const auto slices = 100;
            const auto finish_tm = start_tm + step_tm * slices;

            DynamicObjectsGrid<RegularStationaryDynamicObject<geometry::Rectangle>> grid(
                5.0, start_tm, slices, step_tm);

            std::vector<RegularStationaryDynamicObject<geometry::Rectangle>> objects;
            for (size_t i = 0; i < 20; ++i) {
                objects.emplace_back(
                    geometry::Random<geometry::Rectangle>(rd, {.max_width = 4.0, .max_length = 12.0}));
            }

            grid.SetObjects(objects);

            for (size_t i = 0; i < 1000; ++i) {
                const auto& request = geometry::Random<geometry::Point>(rd);
                const auto request_radius = math::random<double>(rd, {.min = 1.0, .max = 100.0});

                const auto tm = time::Random<time::IntClock::time_point>(
                    rd, {.min = time::ToUint64(start_tm), .max = time::ToUint64(finish_tm)});

                double real_distance_sq =
                    ranges::min(objects | views::transform([&request](const auto& obj) {
                                    return geometry::DistanceSq(obj.GetOriginalGeometry(), request);
                                }));
                math::UpdateMin(real_distance_sq, request_radius * request_radius);

                EXPECT_EQ(
                    real_distance_sq,
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kFast>(tm, request,
                                                                                             request_radius));
                EXPECT_EQ(
                    real_distance_sq,
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kAccurate>(tm, request,
                                                                                                 request_radius));
                EXPECT_EQ(
                    real_distance_sq,
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kOccupancyGrid>(
                        tm, request, request_radius));
            }
        }
    }

    TEST(DynamicObjectsGrid, AbstractStationary) {
        std::mt19937 rd(42);

        for (size_t iteration = 0; iteration < 200; ++iteration) {
            const auto start_tm = time::IntClock::now();
            const auto step_tm = std::chrono::milliseconds(100);
            const auto slices = 100;
            const auto finish_tm = start_tm + step_tm * slices;

            using Geometry = geometry::EfficientDistanceAnyGeometry;
            static_assert(std::is_same_v<geometry::PointOf<Geometry>, geometry::Point>);

            DynamicObjectsGrid<std::shared_ptr<DynamicObject<Geometry>>> grid(
                5.0, start_tm, slices, step_tm);

            std::vector<std::shared_ptr<DynamicObject<Geometry>>> objects;
            for (size_t i = 0; i < 10; ++i) {
                auto rect = geometry::Random<geometry::Rectangle>(rd, {.max_width = 4.0, .max_length = 12.0});
                objects.push_back(std::make_shared<StationaryDynamicObject<Geometry>>(
                    geometry::ConvertToEfficientDistanceRepresentation(rect)));
            }
            for (size_t i = 0; i < 10; ++i) {
                auto circle = geometry::Random<geometry::Circle>(rd, {.min_radius = 0.1, .max_radius = 3.0});
                objects.push_back(std::make_shared<RegularStationaryDynamicObject<Geometry>>(circle));
            }

            grid.SetObjects(objects);

            for (size_t i = 0; i < 1000; ++i) {
                const auto& request = geometry::Random<geometry::Point>(rd);
                const auto request_radius = math::random<double>(rd, {.min = 1.0, .max = 100.0});

                const auto tm = time::Random<time::IntClock::time_point>(
                    rd, {.min = time::ToUint64(start_tm), .max = time::ToUint64(finish_tm)});

                double real_distance_sq = ranges::min(
                    objects | views::transform([&request](const auto& obj) {
                        if (auto s = std::dynamic_pointer_cast<StationaryDynamicObject<Geometry>>(obj)) {
                            return geometry::DistanceSq(s->GetOriginalFastGeometry(), request);
                        } else if (
                            auto r = std::dynamic_pointer_cast<RegularStationaryDynamicObject<Geometry>>(obj)) {
                            return geometry::DistanceSq(r->GetOriginalGeometry(), request);
                        } else {
                            SDC_FAIL("impossible");
                            return std::numeric_limits<double>::infinity();
                        }
                    }));
                math::UpdateMin(real_distance_sq, request_radius * request_radius);

                EXPECT_EQ(
                    real_distance_sq,
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kFast>(tm, request,
                                                                                             request_radius));
                EXPECT_EQ(
                    real_distance_sq,
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kAccurate>(tm, request,
                                                                                                 request_radius));
                EXPECT_EQ(
                    real_distance_sq,
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kOccupancyGrid>(
                        tm, request, request_radius));
            }
        }
    }

    TEST(DynamicObjectsGrid, AbstractStationaryWithReferenceWrapper) {
        std::mt19937 rd(42);

        for (size_t iteration = 0; iteration < 200; ++iteration) {
            const auto start_tm = time::IntClock::now();
            const auto step_tm = std::chrono::milliseconds(100);
            const auto slices = 100;
            const auto finish_tm = start_tm + step_tm * slices;

            using Geometry = std::variant<geometry::Baton, geometry::Circle>;

            DynamicObjectsGrid<std::reference_wrapper<DynamicObject<Geometry>>> grid(
                5.0, start_tm, slices, step_tm);

            std::vector<StationaryDynamicObject<Geometry>> objects_rect;
            std::vector<StationaryDynamicObject<Geometry>> objects_circles;
            for (size_t i = 0; i < 10; ++i) {
                auto rect = geometry::Random<geometry::Baton>(rd, {.min_radius = 0.1, .max_radius = 3.0});
                auto rect_p05 = rect;
                rect_p05.buffer(0.5);
                auto rect_m005 = rect;
                rect_m005.buffer(-0.05);
                objects_rect.emplace_back(rect, rect_p05, rect_m005);
            }
            for (size_t i = 0; i < 10; ++i) {
                auto circle = geometry::Random<geometry::Circle>(rd, {.min_radius = 0.1, .max_radius = 3.0});
                auto circle_p05 = circle;
                circle_p05.buffer(0.5);
                auto circle_m005 = circle;
                circle_m005.buffer(-0.05);
                objects_circles.emplace_back(
                    circle, circle_p05, circle_m005);
            }

            std::vector<std::reference_wrapper<DynamicObject<Geometry>>> objects;
            objects.reserve(objects_rect.size() + objects_circles.size());
            for (auto& obj : objects_rect) {
                objects.push_back(obj);
            }
            for (auto& obj : objects_circles) {
                objects.push_back(obj);
            }

            grid.SetObjects<GeometryAccessType::kAccurate>(objects);

            for (size_t i = 0; i < 1000; ++i) {
                const auto& request = geometry::Random<geometry::Point>(rd);
                const auto request_radius = math::random<double>(rd, {.min = 1.0, .max = 100.0});

                const auto tm = time::Random<time::IntClock::time_point>(
                    rd, {.min = time::ToUint64(start_tm), .max = time::ToUint64(finish_tm)});

                double real_distance_sq = ranges::min(
                    objects | views::transform([&request](const auto& obj) {
                        if (auto s = dynamic_cast<const StationaryDynamicObject<Geometry>*>(&obj.get())) {
                            return geometry::DeepDistanceSq(s->GetOriginalFastGeometry(), request);
                        } else if (
                            auto r =
                                dynamic_cast<const RegularStationaryDynamicObject<Geometry>*>(&obj.get())) {
                            return geometry::DeepDistanceSq(r->GetOriginalGeometry(), request);
                        } else {
                            SDC_FAIL("impossible");
                            return std::numeric_limits<double>::infinity();
                        }
                    }));

                ASSERT_EQ(
                    math::clamp(real_distance_sq, 0.0, math::squared(request_radius)),
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kFast>(tm, request,
                                                                                             request_radius));
                ASSERT_NEAR(
                    math::squared(
                        math::clamp(math::sqrt_with_sign(real_distance_sq) - 0.5, 0.0, request_radius)),
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kAccurate>(
                        tm, request, request_radius),
                    1e-10);
                ASSERT_NEAR(
                    math::squared(
                        math::clamp(math::sqrt_with_sign(real_distance_sq) + 0.05, 0.0, request_radius)),
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kOccupancyGrid>(
                        tm, request, request_radius),
                    1e-10);
            }
        }
    }

    template <typename Geometry>
    struct DynamicLaw {
        Geometry geometry;
        ros::Time start_time;
        geometry::Point start_point;
        geometry::Direction direction;
        double speed;
    };

    template <typename Geometry>
    std::ostream& operator<<(std::ostream& os, const DynamicLaw<Geometry>& law) {
        return os << "DynamicLaw{geometry=" << law.geometry << ", start_time=" << law.start_time
                  << ", start_point=" << law.start_point << ", direction=" << law.direction
                  << ", speed=" << law.speed << "}";
    }

    template <typename ObjectGeometry, typename RequestGeometry>
    double GetDynamicDistanceSq(
        const DynamicLaw<ObjectGeometry>& law, const RequestGeometry& request, ros::Time tm) {
        ros::Duration duration = tm - law.start_time;
        const auto current_point = law.start_point + law.direction * law.speed * duration.toSec();
        const auto shifted_geometry =
            geometry::Translate(law.geometry, geometry::MakeVector(law.start_point, current_point));
        return geometry::DistanceSq(shifted_geometry, request);
    }

    TEST(DynamicObjectsGrid, AbstractMotionNSpline) {
        std::mt19937 rd(42);

        for (size_t iteration = 0; iteration < 200; ++iteration) {
            const auto start_tm = time::IntClock::now();
            const auto step_tm = std::chrono::milliseconds(100);
            const auto slices = 100;
            const auto finish_tm = start_tm + step_tm * slices;

            using Geometry = std::variant<geometry::LazyTransformGeometry<geometry::NormalizedBox>, geometry::Circle>;

            DynamicObjectsGrid<std::shared_ptr<DynamicObject<Geometry>>> grid(
                5.0, start_tm, slices, step_tm);

            std::vector<geometry::LazyTransformGeometry<geometry::NormalizedBox>> objects_stationary_rect;
            std::vector<geometry::Circle> objects_stationary_circles;
            for (size_t i = 0; i < 3; ++i) {
                auto rect = geometry::Random<geometry::Rectangle>(rd, {.max_width = 4.0, .max_length = 12.0});
                objects_stationary_rect.push_back(
                    geometry::Convert<geometry::LazyTransformGeometry<geometry::NormalizedBox>>(rect));
            }
            for (size_t i = 0; i < 3; ++i) {
                auto circle = geometry::Random<geometry::Circle>(rd, {.min_radius = 0.1, .max_radius = 3.0});
                objects_stationary_circles.push_back(circle);
            }

            std::vector<DynamicLaw<Geometry>> objects_dynamic;
            for (size_t i = 0; i < 3; ++i) {
                objects_dynamic.push_back(DynamicLaw<Geometry>{
                    .geometry = geometry::Random<geometry::Circle>(rd, {.min_radius = 0.1, .max_radius = 3.0}),
                    .start_time = time::Random<ros::Time>(
                        rd, {.min = time::ToUint64(start_tm), .max = time::ToUint64(finish_tm)}),
                    .start_point = geometry::Random<geometry::Point>(rd),
                    .direction = geometry::Direction{geometry::Random<geometry::UnitVector>(rd)},
                    .speed = math::random<double>(rd, {.min = 1.0, .max = 10.0})});
            }
            for (size_t i = 0; i < 3; ++i) {
                objects_dynamic.push_back(DynamicLaw<Geometry>{
                    .geometry = geometry::Convert<geometry::LazyTransformGeometry<geometry::NormalizedBox>>(
                        geometry::Random<geometry::Rectangle>(rd, {.max_width = 4.0, .max_length = 12.0})),
                    .start_time = time::Random<ros::Time>(
                        rd, {.min = time::ToUint64(start_tm), .max = time::ToUint64(finish_tm)}),
                    .start_point = geometry::Random<geometry::Point>(rd),
                    .direction = geometry::Direction{geometry::Random<geometry::UnitVector>(rd)},
                    .speed = math::random<double>(rd, {.min = 0.0, .max = 25.0})});
            }

            std::vector<std::shared_ptr<DynamicObject<Geometry>>> objects;
            objects.reserve(
                objects_stationary_rect.size() + objects_stationary_circles.size() +
                objects_dynamic.size());
            for (auto& obj : objects_stationary_rect) {
                objects.push_back(std::make_shared<RegularStationaryDynamicObject<Geometry>>(obj));
            }
            for (auto& obj : objects_stationary_circles) {
                objects.push_back(std::make_shared<StationaryDynamicObject<Geometry>>(obj));
            }
            for (auto& law : objects_dynamic) {
                MotionVSpline motion(
                    law.start_time,
                    {geometry::Convert<geometry::Vector>(law.start_point), law.direction.unitDir() * law.speed});
                objects.push_back(std::make_shared<MotionSplineDynamicObject<Geometry, MotionVSpline>>(
                    motion, law.geometry, geometry::Centroid(law.geometry)));
            }
            grid.SetObjects(objects);

            for (size_t i = 0; i < 1000; ++i) {
                const auto& request = geometry::Random<geometry::Point>(rd);
                const auto request_radius = math::random<double>(rd, {.min = 1.0, .max = 100.0});

                const auto tm = time::Random<time::IntClock::time_point>(
                    rd, {.min = time::ToUint64(start_tm), .max = time::ToUint64(finish_tm)});

                double real_distance_rect_sq = ranges::min(
                    objects_stationary_rect |
                    views::transform([&request](const auto& obj) { return geometry::DistanceSq(obj, request); }));
                double real_distance_circle_sq = ranges::min(
                    objects_stationary_circles |
                    views::transform([&request](const auto& obj) { return geometry::DistanceSq(obj, request); }));
                double real_distance_dynamic_sq = ranges::min(
                    objects_dynamic | views::transform([request, tm](const auto& law) {
                        return GetDynamicDistanceSq(law, request, time::GetRosTimestamp(tm));
                    }));
                double real_distance_sq =
                    math::min(real_distance_rect_sq, real_distance_circle_sq, real_distance_dynamic_sq);

                math::UpdateMin(real_distance_sq, request_radius * request_radius);

                EXPECT_NEAR(
                    real_distance_sq,
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kFast>(
                        tm, request, request_radius),
                    1e-10);
                EXPECT_NEAR(
                    real_distance_sq,
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kAccurate>(
                        tm, request, request_radius),
                    1e-10);
                EXPECT_NEAR(
                    real_distance_sq,
                    grid.GetClosestObjectDistanceSq<kinematics::GeometryAccessType::kOccupancyGrid>(
                        tm, request, request_radius),
                    1e-10);
            }
        }
    }

} // namespace sdc::kinematics
