#pragma once

#include "common/macros.h"


#include "geometry/angle.h"
#include "geometry/normalized_box.h"
#include "geometry/random.h"
#include "geometry/vector.h"
#include "geometry/generic/access_fwd.h"
#include "geometry/generic/concepts/concept_type.h"
#include "geometry/generic/concepts/construct.h"
#include "geometry/generic/distance/distance_base.h"
#include "geometry/generic/envelope.h"
#include "geometry/generic/serialization/serialization_base.h"
#include "geometry/generic/translation.h"
#include "geometry/generic/within.h"

#include <type_traits>

namespace sdc::geometry {

    namespace impl {

        // if you want to add a new type, you must write some tests
        template <typename T>
        concept IsSupportedGeometryInLazyTransformationGeometry =
            IsPoint<T> || IsCircle<T> ||
            RectangleConcept<T> || IsSimplePolygon<T> || IsBaton<T> || IsBox<T> || IsCustomGeometry<T>;

        template <typename T>
        concept IsDereferencable = requires(T a) {
            { *a };
        };

        template <typename T>
        concept IsReferenceWrapper = requires(T a) {
            { a.get() } -> std::same_as<const typename T::type&>;
        };

    } // namespace impl

    // rotation_angle is angle between real_geometry and geometry without shift
    template <typename Geometry>
    class LazyTransformationGeometry {
    public:
        using Scalar = geometry::ScalarOf<Geometry>;

        // ---------------- types ------------------
        LazyTransformationGeometry() = default;

        LazyTransformationGeometry(
            Geometry geometry, const VectorT<Scalar>& geometry_center, const VectorT<Scalar>& shift,
            const DirectionT<Scalar>& rotation_angle) noexcept
            : base_geometry_(std::move(geometry))
            , geometry_center_(geometry_center)
            , shift_(shift)
            , rotation_angle_(rotation_angle)
        {
            static_assert(
                impl::IsSupportedGeometryInLazyTransformationGeometry<decltype(wrapped_geometry())>);
        }

        template <typename R>
            requires(IsRectangle<R> || IsBox<R>)
        explicit LazyTransformationGeometry(R&& rect) noexcept // NOLINT
            requires(std::same_as<Geometry, NormalizedBoxT<PointT<Scalar>>>);

        [[nodiscard]] constexpr const auto& shift() const noexcept {
            return shift_;
        }
        [[nodiscard]] constexpr auto& shift() noexcept {
            return shift_;
        }
        [[nodiscard]] constexpr const auto& rotation_angle() const noexcept {
            return rotation_angle_;
        }
        [[nodiscard]] constexpr auto& rotation_angle() noexcept {
            return rotation_angle_;
        }
        [[nodiscard]] constexpr const auto& geometry_center() const noexcept {
            return geometry_center_;
        }
        [[nodiscard]] constexpr auto& geometry_center() noexcept {
            return geometry_center_;
        }
        [[nodiscard]] constexpr auto real_geometry_center() const noexcept {
            return shift() + geometry_center();
        }
        [[nodiscard]] constexpr const auto& wrapped_geometry() const noexcept {
            if constexpr (impl::IsDereferencable<Geometry>) {
                return *base_geometry_;
            } else if constexpr (impl::IsReferenceWrapper<Geometry>) {
                return base_geometry_.get();
            } else {
                return base_geometry_;
            }
        }

        [[nodiscard]] constexpr auto real_geometry() const noexcept {
            auto geometry_relative_cor = geometry::Translate(wrapped_geometry(), -geometry_center());
            auto rotated_geometry_relative_cor =
                geometry::RotateCounterClockwise(geometry_relative_cor, rotation_angle());
            auto rotated_shifted_geometry =
                geometry::Translate(rotated_geometry_relative_cor, shift() + geometry_center());
            return rotated_shifted_geometry;
        }

    private:

        // ---------------- data -------------------
        Geometry base_geometry_{};

        // ------------ transformation data -------------
        VectorT<Scalar> geometry_center_{};
        VectorT<Scalar> shift_{};
        DirectionT<Scalar> rotation_angle_{};
    };

    template <typename Geometry>
    struct RandomLTGParameters {
        geometry::RandomParametersFor<Geometry> geometry_params{};
        geometry::RandomParametersFor<geometry::VectorT<geometry::ScalarOf<Geometry>>> center_params{};
        geometry::RandomParametersFor<geometry::VectorT<geometry::ScalarOf<Geometry>>> shift_params{};
        /* geometry::RandomParametersFor<geometry::Direction> rotate_params; */
    };

    template <typename T>
    concept IsLazyTransformationGeometry = requires(T t) {
        { t.real_geometry() };
    };

    template <typename Geometry>
    struct GeometryTraits<LazyTransformationGeometry<Geometry>> {
        static constexpr auto concept_type = ConceptType::kCustomGeometry;
        using scalar_type = ScalarOf<Geometry>;
        using point_type = PointOf<Geometry>;
        using random_parameters_type = RandomLTGParameters<Geometry>;
        using geometry_type = LazyTransformationGeometry<Geometry>;

        static constexpr decltype(auto) GetRealGeometry(const geometry_type& r) noexcept {
            return r.real_geometry();
        }

        static constexpr const auto& GetWrappedGeometry(const geometry_type& r) noexcept {
            return r.wrapped_geometry();
        }

        static constexpr const auto& GetRealGeometryCenter(const geometry_type& r) noexcept {
            return r.real_geometry_center();
        }

        template <typename AnyGeometry>
            requires(geometry::DeepDistanceAvailable<AnyGeometry, Geometry>)
        ALWAYS_INLINE static auto DeepDistance(
            const geometry_type& lazy_geometry,
            const AnyGeometry& any_geometry) noexcept {
            const auto& rev_geometry = ReverseTransformationGeometry(lazy_geometry, any_geometry);
            const auto& wrapped_geometry = lazy_geometry.wrapped_geometry();
            return geometry::DeepDistance(rev_geometry, wrapped_geometry);
        }

        template <typename AnyGeometry>
            requires(geometry::DeepDistanceAvailable<AnyGeometry, Geometry>)
        ALWAYS_INLINE static auto DeepDistanceSq(
            const geometry_type& lazy_geometry,
            const AnyGeometry& any_geometry) noexcept {
            const auto& rev_geometry = ReverseTransformationGeometry(lazy_geometry, any_geometry);
            const auto& wrapped_geometry = lazy_geometry.wrapped_geometry();
            return geometry::DeepDistanceSq(rev_geometry, wrapped_geometry);
        }

        template <typename AnyGeometry>
        ALWAYS_INLINE static auto Distance(
            const geometry_type& lazy_geometry,
            const AnyGeometry& any_geometry) noexcept {
            return geometry::Distance(
                ReverseTransformationGeometry(lazy_geometry, any_geometry),
                lazy_geometry.wrapped_geometry());
        }

        template <class T>
        ALWAYS_INLINE static bool Within(
            const geometry_type& lazy_geometry, T&& other_geometry) noexcept {
            return geometry::Within(lazy_geometry.real_geometry(), std::forward<T>(other_geometry));
        }

        template <class T>
        ALWAYS_INLINE static bool Within(
            T&& other_geometry, const geometry_type& lazy_geometry) noexcept {
            return geometry::Within(std::forward<T>(other_geometry), lazy_geometry.real_geometry());
        }

        template <typename AnyGeometry>
        ALWAYS_INLINE static auto DistanceSq(
            const geometry_type& lazy_geometry,
            const AnyGeometry& any_geometry) noexcept {
            return geometry::DistanceSq(
                ReverseTransformationGeometry(lazy_geometry, any_geometry),
                lazy_geometry.wrapped_geometry());
        }

        template <typename AnyGeometry>
        ALWAYS_INLINE static auto OrientedDistance(
            const geometry_type& lazy_geometry,
            const AnyGeometry& any_geometry) noexcept {
            return geometry::OrientedDistance(
                ReverseTransformationGeometry(lazy_geometry, any_geometry),
                lazy_geometry.wrapped_geometry());
        }

        template <typename BoxRefT, typename G1>
        ALWAYS_INLINE PURE constexpr decltype(auto) Expand(
            BoxRefT&& box, const geometry_type& lazy_geometry) noexcept {
            return geometry::Expand(box, lazy_geometry.real_geometry());
        }

        template <IsBox BoxT>
        ALWAYS_INLINE static auto Envelope(const geometry_type& lazy_geometry) noexcept {
            return geometry::Envelope<BoxT>(lazy_geometry.real_geometry());
        }

        template <typename T>
        ALWAYS_INLINE static geometry_type RotateClockwise(
            const geometry_type& lazy_geometry, T&& arg) noexcept {
            geometry_type copy{lazy_geometry};
            copy.rotation_angle() = geometry::RotateClockwise(copy.rotation_angle(), arg);
            copy.shift() = geometry::Add(
                geometry::RotateClockwise(copy.shift() + copy.geometry_center(), arg), -copy.geometry_center());
            return copy;
        }

        template <typename T>
        ALWAYS_INLINE static geometry_type RotateCounterClockwise(
            const geometry_type& lazy_geometry, T&& arg) noexcept {
            geometry_type copy{lazy_geometry};
            copy.rotation_angle() = geometry::RotateCounterClockwise(copy.rotation_angle(), arg);
            copy.shift() = geometry::Add(
                geometry::RotateCounterClockwise(copy.shift() + copy.geometry_center(), arg),
                -copy.geometry_center());
            return copy;
        }

        ALWAYS_INLINE static geometry_type TurnLeft(
            const geometry_type& lazy_geometry) noexcept {
            geometry_type copy{lazy_geometry};
            copy.rotation_angle() = geometry::TurnLeft(copy.rotation_angle());
            copy.shift() = geometry::Add(
                geometry::TurnLeft(copy.shift() + copy.geometry_center()),
                -copy.geometry_center());
            return copy;
        }

        ALWAYS_INLINE static geometry_type TurnRight(
            const geometry_type& lazy_geometry) noexcept {
            geometry_type copy{lazy_geometry};
            copy.rotation_angle() = geometry::TurnRight(copy.rotation_angle());
            copy.shift() = geometry::Add(
                geometry::TurnRight(copy.shift() + copy.geometry_center()),
                -copy.geometry_center());
            return copy;
        }

        template <GeometryConcept T, typename Scalar>
        ALWAYS_INLINE static bool IsApprox(
            const geometry_type& lazy_geometry,
            T&& any_geometry,
            const Scalar& precision) noexcept {
            return geometry::IsApprox(
                ReverseTransformationGeometry(lazy_geometry, any_geometry),
                lazy_geometry.wrapped_geometry(), precision);
        }

        template <typename T>
        ALWAYS_INLINE static decltype(auto) Scale(
            const geometry_type& lazy_geometry, T&& arg) noexcept {
            return geometry::Scale(lazy_geometry.real_geometry(), arg);
        }

        template <typename T>
        ALWAYS_INLINE static geometry_type Translate(
            const geometry_type& lazy_geometry, T&& arg) noexcept {
            geometry_type copy{lazy_geometry};
            copy.shift() = geometry::Add(copy.shift(), arg);
            return copy;
        }

        template <typename Rng, typename Parameters>
        ALWAYS_INLINE PURE static constexpr auto Random(
            Rng& rng, const Parameters& params) noexcept {
            return LazyTransformationGeometry{
                geometry::Random<Geometry>(rng, params.geometry_params),
                geometry::Random<geometry::VectorT<scalar_type>>(rng, params.center_params),
                geometry::Random<geometry::VectorT<scalar_type>>(rng, params.shift_params),
                geometry::DirectionT<scalar_type>::FromDir(geometry::RandAngle(rng))};
        }

        ALWAYS_INLINE static point_type Centroid(const geometry_type& geometry) noexcept {
            return geometry::Centroid(geometry.real_geometry());
        }

        template <SerializationType type>
        ALWAYS_INLINE static std::ostream& Serialize(
            std::ostream& out, const geometry_type& geometry) noexcept {
            return geometry::Serialize<type>(out, geometry.real_geometry());
        }

    private:
        template <typename AnyGeometry>
        ALWAYS_INLINE static decltype(auto) ReverseTransformationGeometry(
            const geometry_type& lazy_geometry,
            const AnyGeometry& any_geometry) noexcept {
            FLOAT_OPTIMIZE

            const auto shifted_geometry_relative_cor = geometry::Translate(
                any_geometry,
                -lazy_geometry.shift() - lazy_geometry.geometry_center());
            const auto rotated_shifted_geometry_relative_cor = geometry::RotateCounterClockwise(
                shifted_geometry_relative_cor, -lazy_geometry.rotation_angle());
            const auto rotated_shifted_geometry = geometry::Translate(
                rotated_shifted_geometry_relative_cor, lazy_geometry.geometry_center());

            return rotated_shifted_geometry;
        }
    };

    template <IsConcept To, IsLazyTransformationGeometry From>
    struct convert_impl<To, From> {
        static constexpr auto Convert(const From& v) {
            return geometry::Convert<To>(v.real_geometry());
        }
    };

} // namespace sdc::geometry
