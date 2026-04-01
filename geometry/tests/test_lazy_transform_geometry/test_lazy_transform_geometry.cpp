#include "geometry/angle.h"
#include "geometry/baton.h"
#include "geometry/box.h"
#include "geometry/circle.h"
#include "geometry/generic/concepts/basic.h"
#include "geometry/generic/concepts/concept_type.h"
#include "geometry/generic/concepts/concept_type_dispatch.h"
#include "geometry/generic/convert/convert_base.h"
#include "geometry/generic/distance/distance_base.h"
#include "geometry/generic/distance/segment_segment.h"
#include "geometry/generic/envelope.h"
#include "geometry/generic/equals/equals_base.h"
#include "geometry/generic/random/random_base.h"
#include "geometry/generic/rotation.h"
#include "geometry/generic/scale.h"
#include "geometry/generic/translation.h"
#include "geometry/lazy_transform_geometry.h"
#include "geometry/matchers.h"
#include "geometry/normalized_box.h"
#include "geometry/point.h"
#include "geometry/polygon.h"
#include "geometry/random.h"
#include "geometry/rectangle.h"
#include "geometry/rectangle_precalc.h"
#include "geometry/rotation.h"
#include "geometry/unit_vector.h"
#include "geometry/vector.h"
#include "geometry/wrappers.h"
#include <math/concepts/basic.h>
#include <math/random.h>
#include <optional>
#include "common/macros.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <sstream>
#include <type_traits>
#include <utility>

namespace sdc::geometry {

    template <typename FromGeometry, typename LTG, typename FLTG>
    void CheckCorrectDistanceLTG(
        const LTG& ltg, const FromGeometry& from, const FLTG& false_ltg,
        std::optional<double>& answer_dist, std::optional<double>& answer_deep_dist,
        std::optional<double>& answer_or_dist) {
        auto check_part = [&](auto lambda_dist, std::optional<double>& answer) {
            double val = lambda_dist(ltg, from);
            double com_val = lambda_dist(from, ltg);

            EXPECT_GEOM_EQ(val, com_val);
            if (answer) {
                EXPECT_GEOM_EQ(val, *answer);
            } else {
                answer = val;
            }
            double with_real_val = lambda_dist(ltg.real_geometry(), from);
            EXPECT_GEOM_EQ(val, with_real_val);

            double false_val = lambda_dist(false_ltg, from);
            EXPECT_FALSE(val == false_val && (val != 0 || false_val != 0));
        };

        using wrapped_type = std::remove_cvref<decltype(ltg.wrapped_geometry())>;
        using from_type = std::remove_cvref<FromGeometry>;

        if constexpr (geometry::traits::HasDistanceGenericImpl<wrapped_type, from_type>) {
            check_part([](const auto& l, const auto& r) { return geometry::Distance(l, r); }, answer_dist);
        }

        if constexpr (geometry::traits::HasDeepDistanceGenericImpl<wrapped_type, from_type>) {
            check_part(
                [](const auto& l, const auto& r) { return geometry::DeepDistance(l, r); }, answer_deep_dist);
        }

        if constexpr (geometry::traits::HasDistanceSqGenericImpl<wrapped_type, from_type>) {
            std::optional<double> answer_dist_sq =
                (answer_dist ? std::optional<double>(*answer_dist * (*answer_dist)) : std::nullopt);
            check_part([](const auto& l, const auto& r) { return geometry::DistanceSq(l, r); }, answer_dist_sq);
        }

        if constexpr (geometry::traits::HasDeepDistanceSqGenericImpl<wrapped_type, from_type>) {
            std::optional<double> answer_deep_dist_sq =
                (answer_deep_dist ? std::optional<double>(*answer_deep_dist * math::abs(*answer_deep_dist)) : std::nullopt);
            check_part(
                [](const auto& l, const auto& r) { return geometry::DeepDistanceSq(l, r); },
                answer_deep_dist_sq);
        }

        if constexpr (geometry::traits::HasOrientedDistanceGenericImpl<wrapped_type, from_type>) {
            check_part(
                [](const auto& l, const auto& r) { return geometry::OrientedDistance(l, r); }, answer_or_dist);
        }
    }

    template <typename FromGeometry, typename ToGeometry>
    void CheckCorrectDistance(
        std::mt19937_64& rd, const FromGeometry& from, const ToGeometry& geometry,
        const geometry::Vector& geometry_center, const geometry::Vector& shift, const geometry::Direction& angle,
        std::optional<double> answer_dist = std::nullopt,
        std::optional<double> answer_deep_dist = std::nullopt,
        std::optional<double> answer_or_dist = std::nullopt) {
        auto rd_false_geometry = geometry::Random<std::remove_cvref_t<ToGeometry>>(rd);
        geometry::LazyTransformGeometry false_ltg{
            &rd_false_geometry, geometry::Random<geometry::Vector>(rd), geometry::Random<geometry::Vector>(rd),
            geometry::RandAngle(rd).toVector()};

        geometry::LazyTransformGeometry ltg1{&geometry, geometry_center, shift, angle};
        CheckCorrectDistanceLTG(ltg1, from, false_ltg, answer_dist, answer_deep_dist, answer_or_dist);

        geometry::LazyTransformGeometry ltg2{geometry, geometry_center, shift, angle};
        CheckCorrectDistanceLTG(ltg2, from, false_ltg, answer_dist, answer_deep_dist, answer_or_dist);

        geometry::LazyTransformGeometry ltg3{
            std::make_shared<ToGeometry>(geometry), geometry_center, shift, angle};
        CheckCorrectDistanceLTG(ltg3, from, false_ltg, answer_dist, answer_deep_dist, answer_or_dist);

        geometry::LazyTransformGeometry ltg4{
            std::make_unique<ToGeometry>(geometry), geometry_center, shift, angle};
        CheckCorrectDistanceLTG(ltg4, from, false_ltg, answer_dist, answer_deep_dist, answer_or_dist);

        geometry::LazyTransformGeometry ltg5{
            std::reference_wrapper<const ToGeometry>(geometry), geometry_center, shift, angle};
        CheckCorrectDistanceLTG(ltg4, from, false_ltg, answer_dist, answer_deep_dist, answer_or_dist);

        geometry::LazyTransformGeometry ltg6{
            std::make_optional<ToGeometry>(geometry), geometry_center, shift, angle};
        CheckCorrectDistanceLTG(ltg4, from, false_ltg, answer_dist, answer_deep_dist, answer_or_dist);
    }

    geometry::Vector RotateVector(geometry::Vector vec, geometry::Direction direction) {
        auto cos_angle = direction.unitDir().x();
        auto sin_angle = direction.unitDir().y();
        return {vec.x() * cos_angle - vec.y() * sin_angle, vec.x() * sin_angle + vec.y() * cos_angle};
    }

    template <typename FromGeometry, typename InnerGeometry>
    void TestLazyTransformGeometry(std::mt19937_64 rd) {
        using geometry::ConceptType;
        size_t iterations = 1'000;
        while (iterations--) {
            const auto rd_geometry = geometry::Random<InnerGeometry>(rd);
            const geometry::Vector geometry_center = [&rd_geometry]() {
                if constexpr (geometry::IsBaton<InnerGeometry>) {
                    return (rd_geometry.start() + rd_geometry.finish()) / 2. - geometry::Point::Zero();
                } else if constexpr (geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kSimplePolygon) {
                    return rd_geometry.front() - geometry::Point::Zero();
                } else if constexpr (geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kRectangle) {
                    return rd_geometry.corner() - geometry::Point::Zero();
                } else if constexpr (geometry::IsCircle<InnerGeometry>) {
                    return rd_geometry.center() - geometry::Point::Zero();
                } else if constexpr (geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kPoint) {
                    return rd_geometry - geometry::Point::Zero();
                } else if constexpr (geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kBox) {
                    return rd_geometry.center() - geometry::Point::Zero();
                } else {
                    SDC_FAIL();
                }
            }();
            const auto rd_req = geometry::Random<FromGeometry>(rd);
            const auto rd_angle = geometry::RandAngle(rd).toVector();
            const auto rd_shift = geometry::Random<geometry::Vector>(rd);

            {
                auto get_ans_geometry = [&]() -> std::optional<InnerGeometry> {
                    if constexpr (
                        geometry::IsBaton<InnerGeometry> ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kSimplePolygon) {
                        return std::nullopt;
                    } else if constexpr (geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kRectangle) {
                        geometry::Point ans_center_shifted = (geometry::Point::Zero() + geometry_center + rd_shift);
                        return geometry::Rectangle{
                            ans_center_shifted, RotateVector(rd_geometry.lengthDir(), rd_angle),
                            RotateVector(rd_geometry.widthDir(), rd_angle)};
                    } else if constexpr (geometry::IsCircle<InnerGeometry>) {
                        geometry::Point ans_center_shifted = (geometry::Point::Zero() + geometry_center + rd_shift);
                        geometry::Circle ans_shifted{ans_center_shifted, rd_geometry.radius()};
                        return ans_shifted;
                    } else if constexpr (geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kPoint) {
                        geometry::Point ans_center_shifted = (geometry::Point::Zero() + geometry_center + rd_shift);
                        return ans_center_shifted;
                    } else {
                        SDC_FAIL();
                    }
                };
                std::optional<double> ans_dist = [&]() -> std::optional<double> {
                    if constexpr (
                        geometry::IsBaton<InnerGeometry> ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kSimplePolygon ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kBox) {
                        return std::nullopt;
                    } else if constexpr ( // NOLINT
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kRectangle ||
                        geometry::IsCircle<InnerGeometry> ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kPoint) {
                        auto ans = get_ans_geometry();
                        return geometry::Distance(rd_req, *VERIFY(ans));
                    } else {
                        SDC_FAIL();
                    }
                }();
                std::optional<double> ans_deep_dist = [&]() -> std::optional<double> {
                    if constexpr (
                        geometry::IsBaton<InnerGeometry> ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kSimplePolygon ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kRectangle ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kBox) {
                        return std::nullopt;
                    } else if constexpr ( // NOLINT
                        geometry::IsCircle<InnerGeometry> ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kPoint) {
                        auto ans = get_ans_geometry();
                        return geometry::DeepDistance(rd_req, *VERIFY(ans));
                    } else {
                        SDC_FAIL();
                    }
                }();
                CheckCorrectDistance(
                    rd, rd_req, rd_geometry, geometry_center, rd_shift, rd_angle, ans_dist, ans_deep_dist);
            }

            // rd_center ckecking
            {
                const std::optional<geometry::Vector> rd_center = [&]() -> std::optional<geometry::Vector> {
                    if constexpr (geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kPoint) {
                        return rd_geometry - geometry::Point::Zero();
                    } else if constexpr ( // NOLINT
                        geometry::IsBaton<InnerGeometry> ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kSimplePolygon ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kRectangle ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kBox) {
                        return std::nullopt;
                    } else if constexpr (geometry::IsCircle<InnerGeometry>) {
                        const geometry::Vector rd_vec_from_center =
                            geometry::Random<geometry::Vector>(rd).normalized() *
                            math::random<double>(rd, {.min = 0., .max = rd_geometry.radius()});
                        return geometry_center + rd_vec_from_center;
                    } else {
                        SDC_FAIL();
                    }
                }();
                if (!rd_center) {
                    continue;
                }
                auto get_ans_geometry = [&]() -> std::optional<InnerGeometry> {
                    if constexpr (geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kPoint) {
                        return geometry::Point{geometry::GetX(*rd_center), geometry::GetY(*rd_center)};
                    } else if constexpr ( // NOLINT
                        geometry::IsBaton<InnerGeometry> ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kSimplePolygon ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kPoint ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kRectangle ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kBox) {
                        return std::nullopt;
                    } else if constexpr (geometry::IsCircle<InnerGeometry>) {
                        const geometry::Vector rd_vec_from_center = *rd_center - geometry_center;
                        geometry::Vector vec_from_center_rotate = RotateVector(rd_vec_from_center, rd_angle);
                        geometry::Vector rotate_shift = rd_vec_from_center - vec_from_center_rotate;
                        geometry::Vector ans_center_shifted =
                            *rd_center + rd_shift - rd_vec_from_center + rotate_shift;
                        geometry::Circle ans_shifted =
                            geometry::Circle{{ans_center_shifted.x(), ans_center_shifted.y()}, rd_geometry.radius()};
                        return ans_shifted;
                    } else {
                        SDC_FAIL();
                    }
                };
                std::optional<double> ans_dist = [&]() -> std::optional<double> {
                    if constexpr (
                        geometry::IsBaton<InnerGeometry> ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kSimplePolygon ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kRectangle ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kPoint ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kBox) {
                        return std::nullopt;
                    } else if constexpr (geometry::IsCircle<InnerGeometry>) {
                        auto ans = get_ans_geometry();
                        return geometry::Distance(rd_req, *VERIFY(ans));
                    } else {
                        SDC_FAIL();
                    }
                }();
                std::optional<double> ans_deep_dist = [&]() -> std::optional<double> {
                    if constexpr (
                        geometry::IsBaton<InnerGeometry> ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kSimplePolygon ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kRectangle ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kPoint ||
                        geometry::ConceptTypeOf<InnerGeometry> == ConceptType::kBox) {
                        return std::nullopt;
                    } else if constexpr (geometry::IsCircle<InnerGeometry>) {
                        auto ans = get_ans_geometry();
                        return geometry::DeepDistance(rd_req, *VERIFY(ans));
                    } else {
                        SDC_FAIL();
                    }
                }();
                CheckCorrectDistance(
                    rd, rd_req, rd_geometry, *rd_center, rd_shift, rd_angle, ans_dist, ans_deep_dist);
            }
            {
                const auto& ltg_rnd = geometry::Random<geometry::LazyTransformGeometry<InnerGeometry>>(rd);
                const auto& rnd_box = geometry::Random<geometry::Box>(rd);
                const auto& rnd_dir = geometry::RandAngle(rd);
                const auto& rnd_double = math::random<double>(rd, {0.1, 10.0});
                const auto& rnd_vec = geometry::Random<geometry::Vector>(rd);

                ASSERT_TRUE(geometry::Equals(
                    geometry::Expand(rnd_box, ltg_rnd), geometry::Expand(rnd_box, ltg_rnd.real_geometry())));
                ASSERT_TRUE(geometry::Equals(
                    geometry::Envelope<geometry::Box>(ltg_rnd),
                    geometry::Envelope<geometry::Box>(ltg_rnd.real_geometry())));
                ASSERT_TRUE(geometry::Equals(
                    geometry::RotateClockwise(ltg_rnd, rnd_dir),
                    geometry::RotateClockwise(ltg_rnd.real_geometry(), rnd_dir)));
                ASSERT_TRUE(geometry::Equals(
                    geometry::RotateCounterClockwise(ltg_rnd, rnd_dir),
                    geometry::RotateCounterClockwise(ltg_rnd.real_geometry(), rnd_dir)));
                ASSERT_TRUE(geometry::Equals(
                    geometry::TurnLeft(ltg_rnd),
                    geometry::TurnLeft(ltg_rnd.real_geometry())));
                ASSERT_TRUE(geometry::Equals(
                    geometry::TurnRight(ltg_rnd),
                    geometry::TurnRight(ltg_rnd.real_geometry())));
                ASSERT_TRUE(geometry::IsApprox(ltg_rnd, ltg_rnd.real_geometry(), 1e-4));
                ASSERT_TRUE(geometry::IsApprox(ltg_rnd, ltg_rnd, 1e-4));
                ASSERT_TRUE(geometry::Equals(
                    geometry::Scale(ltg_rnd, rnd_double),
                    geometry::Scale(ltg_rnd.real_geometry(), rnd_double)));
                ASSERT_TRUE(geometry::Equals(
                    geometry::Translate(ltg_rnd, rnd_vec),
                    geometry::Translate(ltg_rnd.real_geometry(), rnd_vec)));

                const auto& fltg_rnd = geometry::Random<geometry::LazyTransformGeometry<InnerGeometry>>(rd);
                ASSERT_FALSE(geometry::Equals(
                    geometry::Envelope<geometry::Box>(ltg_rnd),
                    geometry::Envelope<geometry::Box>(fltg_rnd.real_geometry())));
                ASSERT_FALSE(geometry::Equals(
                    geometry::RotateClockwise(ltg_rnd, rnd_dir),
                    geometry::RotateClockwise(fltg_rnd.real_geometry(), rnd_dir)));
                ASSERT_FALSE(geometry::Equals(
                    geometry::RotateCounterClockwise(ltg_rnd, rnd_dir),
                    geometry::RotateCounterClockwise(fltg_rnd.real_geometry(), rnd_dir)));
                ASSERT_FALSE(geometry::Equals(
                    geometry::TurnLeft(ltg_rnd),
                    geometry::TurnLeft(fltg_rnd.real_geometry())));
                ASSERT_FALSE(geometry::Equals(
                    geometry::TurnRight(ltg_rnd),
                    geometry::TurnRight(fltg_rnd.real_geometry())));
                ASSERT_FALSE(geometry::IsApprox(ltg_rnd, fltg_rnd.real_geometry(), 1e-4));
                ASSERT_FALSE(geometry::IsApprox(ltg_rnd, fltg_rnd, 1e-4));
                ASSERT_FALSE(geometry::Equals(
                    geometry::Scale(ltg_rnd, rnd_double),
                    geometry::Scale(fltg_rnd.real_geometry(), rnd_double)));
                ASSERT_FALSE(geometry::Equals(
                    geometry::Translate(ltg_rnd, rnd_vec),
                    geometry::Translate(fltg_rnd.real_geometry(), rnd_vec)));
            }
        }
    }

    TEST(DistanceBetweenLazyTransformPoint, Test) {
        std::mt19937_64 rd;
        rd.seed(43);
        TestLazyTransformGeometry<geometry::Point, geometry::Point>(rd);
        TestLazyTransformGeometry<geometry::Point, geometry::Circle>(rd);
        TestLazyTransformGeometry<geometry::Point, geometry::Rectangle>(rd);
        TestLazyTransformGeometry<geometry::Point, geometry::Box>(rd);
        TestLazyTransformGeometry<geometry::Point, geometry::SimplePolygon>(rd);
        TestLazyTransformGeometry<geometry::Point, geometry::Baton>(rd);
    }

    TEST(DistanceBetweenLazyTransformGeometryCircle, Test) {
        std::mt19937_64 rd;
        rd.seed(43);
        TestLazyTransformGeometry<geometry::Circle, geometry::Point>(rd);
        TestLazyTransformGeometry<geometry::Circle, geometry::Circle>(rd);
        TestLazyTransformGeometry<geometry::Circle, geometry::Rectangle>(rd);
        TestLazyTransformGeometry<geometry::Circle, geometry::Box>(rd);
        TestLazyTransformGeometry<geometry::Circle, geometry::SimplePolygon>(rd);
        TestLazyTransformGeometry<geometry::Circle, geometry::Baton>(rd);
    }

    TEST(DistanceBetweenLazyTransformGeometryRectangle, Test) {
        std::mt19937_64 rd;
        rd.seed(43);
        TestLazyTransformGeometry<geometry::Rectangle, geometry::Point>(rd);
        TestLazyTransformGeometry<geometry::Rectangle, geometry::Circle>(rd);
        TestLazyTransformGeometry<geometry::Rectangle, geometry::Rectangle>(rd);
        TestLazyTransformGeometry<geometry::Rectangle, geometry::Box>(rd);
        TestLazyTransformGeometry<geometry::Rectangle, geometry::SimplePolygon>(rd);
        TestLazyTransformGeometry<geometry::Rectangle, geometry::Baton>(rd);
    }

    TEST(DistanceBetweenLazyTransformGeometrySimplePolygon, Test) {
        std::mt19937_64 rd;
        rd.seed(43);
        TestLazyTransformGeometry<geometry::SimplePolygon, geometry::Point>(rd);
        TestLazyTransformGeometry<geometry::SimplePolygon, geometry::Circle>(rd);
        TestLazyTransformGeometry<geometry::SimplePolygon, geometry::Rectangle>(rd);
        TestLazyTransformGeometry<geometry::SimplePolygon, geometry::Box>(rd);
        TestLazyTransformGeometry<geometry::SimplePolygon, geometry::SimplePolygon>(rd);
        TestLazyTransformGeometry<geometry::SimplePolygon, geometry::Baton>(rd);
    }

    TEST(DistanceBetweenLazyTransformGeometryBaton, Test) {
        std::mt19937_64 rd;
        rd.seed(43);
        TestLazyTransformGeometry<geometry::Baton, geometry::Point>(rd);
        TestLazyTransformGeometry<geometry::Baton, geometry::Circle>(rd);
        TestLazyTransformGeometry<geometry::Baton, geometry::Rectangle>(rd);
        TestLazyTransformGeometry<geometry::Baton, geometry::Box>(rd);
        TestLazyTransformGeometry<geometry::Baton, geometry::SimplePolygon>(rd);
        TestLazyTransformGeometry<geometry::Baton, geometry::Baton>(rd);
    }

    TEST(SimpleDistanceBetweenLazyTransformGeometryCircleAndPoint, Test) {
        std::mt19937_64 rd;
        rd.seed(43);
        constexpr auto req = geometry::Point{0, 2};

        constexpr auto geometry = geometry::Circle{geometry::Point{2, 2}, 1};
        constexpr geometry::Vector vec_from_center{0, -1};
        constexpr geometry::Vector geometry_center =
            (geometry.center() - geometry::Point::Zero()) + vec_from_center;
        constexpr auto shift = geometry::Vector{8, 1};

        auto process_angle = [&](geometry::Direction direction, double ans_dist) {
            CheckCorrectDistance(rd, req, geometry, geometry_center, shift, direction, ans_dist, ans_dist);
        };
        process_angle(geometry::Direction::PI_2(), 8);
        process_angle(-geometry::Direction::PI_2(), 10);
        process_angle(geometry::Direction::PI(), math::sqrt(101.) - 1.);
        process_angle(-geometry::Direction::PI(), math::sqrt(101.) - 1.);
    }

    TEST(OutputToStream, Test) {
        const auto box = geometry::Box{{0.1, 0.2}, {10.0, 15.0}};
        const auto lazy = geometry::Convert<LazyTransformGeometry<NormalizedBox>>(box);
        std::stringstream ss;
        ss << lazy;
        ASSERT_EQ(
            ss.str(),
            "{{0.09999999999999964, 0.20000000000000018}, "  // corner
            "{9.90000000000000036, 0.00000000000000000}, "   // length
            "{0.00000000000000000, 14.80000000000000071}}"); // width
    }

} // namespace sdc::geometry
