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

        // if you want to add unsupported type, you must write some tests
        template <typename T>
        concept IsSupportGeometryInLazyTransformGeometry =
            IsPoint<T> || IsCircle<T> ||
            RectangleConcept<T> || IsSimplePolygon<T> || IsBaton<T> || IsBox<T> || IsCustomGeometry<T>;

        template <typename T>
        concept IsDereferencable = requires(T a) {
            { *a };
        };

        template <typename T>
        concept IsRefereenceWrapper = requires(T a) {
            { a.get() } -> std::same_as<const typename T::type&>;
        };

    } // namespace impl

    // class accepts dereferencable class, by value and const&, you must take care of validity and
    // object's time to life in case const* or const&.
    // LazyTransformGeometry<geometry::MeshGeometry>  --  value
    // LazyTransformGeometry<std::shared_ptr<MeshGeometry>>  --  smart pointer
    // LazyTransformGeometry<const geometry::MeshGeometry&>  --  const reference
    // LazyTransformGeometry<std::reference_wrapper<const geometry::MeshGeometry>>  --  reference_wrapper
    // LazyTransformGeometry<const geometry::MeshGeometry*>  --  const pointer
    // rotate_angle is angle between real_geometry and geometry without shift
    template <typename Geometry>
    class LazyTransformGeometry {
    public:
        using Scalar = geometry::ScalarOf<Geometry>;

        // ---------------- types ------------------
        LazyTransformGeometry() = default;

        LazyTransformGeometry(
            Geometry geometry, const VectorT<Scalar>& geometry_center, const VectorT<Scalar>& shift,
            const DirectionT<Scalar>& rotate_angle) noexcept
            : geometry_(std::move(geometry))
            , geometry_center_(geometry_center)
            , shift_(shift)
            , rotate_angle_(rotate_angle)
        {
            static_assert(
                impl::IsSupportGeometryInLazyTransformGeometry<decltype(wrapped_geometry())>);
        }

        template <typename R>
            requires(IsRectangle<R> || IsBox<R>)
        explicit LazyTransformGeometry(R&& rect) noexcept // NOLINT
            requires(std::same_as<Geometry, NormalizedBoxT<PointT<Scalar>>>);

        [[nodiscard]] constexpr const auto& shift() const noexcept {
            return shift_;
        }
        [[nodiscard]] constexpr auto& shift() noexcept {
            return shift_;
        }
        [[nodiscard]] constexpr const auto& rotate_angle() const noexcept {
            return rotate_angle_;
        }
        [[nodiscard]] constexpr auto& rotate_angle() noexcept {
            return rotate_angle_;
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
                return *geometry_;
            } else if constexpr (impl::IsRefereenceWrapper<Geometry>) {
                return geometry_.get();
            } else {
                return geometry_;
            }
        }

        [[nodiscard]] constexpr auto real_geometry() const noexcept {
            auto geometry_relative_cor = geometry::Translate(wrapped_geometry(), -geometry_center());
            auto rotated_geometry_relative_cor =
                geometry::RotateCounterClockwise(geometry_relative_cor, rotate_angle());
            auto rotated_shifted_geometry =
                geometry::Translate(rotated_geometry_relative_cor, shift() + geometry_center());
            return rotated_shifted_geometry;
        }

    private:
        friend class boost::serialization::access;

        // boost serialization
        template <typename Archive>
        void serialize(Archive& ar, const unsigned int /*version*/) noexcept {
            ar & geometry_ & geometry_center_ & shift_ & rotate_angle_;
        }

        // ---------------- data -------------------
        Geometry geometry_{};

        // ------------ transform data -------------
        VectorT<Scalar> geometry_center_{};
        VectorT<Scalar> shift_{};
        DirectionT<Scalar> rotate_angle_{};
    };

    template <typename Geometry>
    struct RandomLTGParameters {
        geometry::RandomParametersFor<Geometry> geom_params{};
        geometry::RandomParametersFor<geometry::VectorT<geometry::ScalarOf<Geometry>>> center_params{};
        geometry::RandomParametersFor<geometry::VectorT<geometry::ScalarOf<Geometry>>> shift_params{};
        /* geometry::RandomParametersFor<geometry::Direction> rotate_params; */
    };

    template <typename T>
    concept IsLazyTransformGeometry = requires(T t) {
        { t.real_geometry() };
    };

    template <typename Geometry>
    struct GeometryTraits<LazyTransformGeometry<Geometry>> {
        static constexpr auto concept_type = ConceptType::kCustomGeometry;
        using scalar_type = ScalarOf<Geometry>;
        using point_type = PointOf<Geometry>;
        using random_parameters_type = RandomLTGParameters<Geometry>;
        using geometry_type = LazyTransformGeometry<Geometry>;

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
            const geometry_type& lazy_rotate_shift_geometry,
            const AnyGeometry& any_geometry) noexcept {
            const auto& rev_geometry = ReverseTransformGeometry(lazy_rotate_shift_geometry, any_geometry);
            const auto& wrapped_geometry = lazy_rotate_shift_geometry.wrapped_geometry();
            return geometry::DeepDistance(rev_geometry, wrapped_geometry);
        }

        template <typename AnyGeometry>
            requires(geometry::DeepDistanceAvailable<AnyGeometry, Geometry>)
        ALWAYS_INLINE static auto DeepDistanceSq(
            const geometry_type& lazy_rotate_shift_geometry,
            const AnyGeometry& any_geometry) noexcept {
            const auto& rev_geometry = ReverseTransformGeometry(lazy_rotate_shift_geometry, any_geometry);
            const auto& wrapped_geometry = lazy_rotate_shift_geometry.wrapped_geometry();
            return geometry::DeepDistanceSq(rev_geometry, wrapped_geometry);
        }

        template <typename AnyGeometry>
        ALWAYS_INLINE static auto Distance(
            const geometry_type& lazy_rotate_shift_geometry,
            const AnyGeometry& any_geometry) noexcept {
            return geometry::Distance(
                ReverseTransformGeometry(lazy_rotate_shift_geometry, any_geometry),
                lazy_rotate_shift_geometry.wrapped_geometry());
        }

        template <class T>
        ALWAYS_INLINE static bool Within(
            const geometry_type& lazy_rotate_shift_geometry, T&& geometry2) noexcept {
            return geometry::Within(lazy_rotate_shift_geometry.real_geometry(), std::forward<T>(geometry2));
        }

        template <class T>
        ALWAYS_INLINE static bool Within(
            T&& geometry2, const geometry_type& lazy_rotate_shift_geometry) noexcept {
            return geometry::Within(std::forward<T>(geometry2), lazy_rotate_shift_geometry.real_geometry());
        }

        template <typename AnyGeometry>
        ALWAYS_INLINE static auto DistanceSq(
            const geometry_type& lazy_rotate_shift_geometry,
            const AnyGeometry& any_geometry) noexcept {
            return geometry::DistanceSq(
                ReverseTransformGeometry(lazy_rotate_shift_geometry, any_geometry),
                lazy_rotate_shift_geometry.wrapped_geometry());
        }

        template <typename AnyGeometry>
        ALWAYS_INLINE static auto OrientedDistance(
            const geometry_type& lazy_rotate_shift_geometry,
            const AnyGeometry& any_geometry) noexcept {
            return geometry::OrientedDistance(
                ReverseTransformGeometry(lazy_rotate_shift_geometry, any_geometry),
                lazy_rotate_shift_geometry.wrapped_geometry());
        }

        template <typename BoxRefT, typename G1>
        ALWAYS_INLINE PURE constexpr decltype(auto) Expand(
            BoxRefT&& box, const geometry_type& lazy_rotate_shift_geometry) noexcept {
            return geometry::Expand(box, lazy_rotate_shift_geometry.real_geometry());
        }

        template <IsBox BoxT>
        ALWAYS_INLINE static auto Envelope(const geometry_type& lazy_rotate_shift_geometry) noexcept {
            return geometry::Envelope<BoxT>(lazy_rotate_shift_geometry.real_geometry());
        }

        template <typename T>
        ALWAYS_INLINE static geometry_type RotateClockwise(
            const geometry_type& lazy_rotate_shift_geometry, T&& arg) noexcept {
            geometry_type copy{lazy_rotate_shift_geometry};
            copy.rotate_angle() = geometry::RotateClockwise(copy.rotate_angle(), arg);
            copy.shift() = geometry::Add(
                geometry::RotateClockwise(copy.shift() + copy.geometry_center(), arg), -copy.geometry_center());
            return copy;
        }

        template <typename T>
        ALWAYS_INLINE static geometry_type RotateCounterClockwise(
            const geometry_type& lazy_rotate_shift_geometry, T&& arg) noexcept {
            geometry_type copy{lazy_rotate_shift_geometry};
            copy.rotate_angle() = geometry::RotateCounterClockwise(copy.rotate_angle(), arg);
            copy.shift() = geometry::Add(
                geometry::RotateCounterClockwise(copy.shift() + copy.geometry_center(), arg),
                -copy.geometry_center());
            return copy;
        }

        ALWAYS_INLINE static geometry_type TurnLeft(
            const geometry_type& lazy_rotate_shift_geometry) noexcept {
            geometry_type copy{lazy_rotate_shift_geometry};
            copy.rotate_angle() = geometry::TurnLeft(copy.rotate_angle());
            copy.shift() = geometry::Add(
                geometry::TurnLeft(copy.shift() + copy.geometry_center()),
                -copy.geometry_center());
            return copy;
        }

        ALWAYS_INLINE static geometry_type TurnRight(
            const geometry_type& lazy_rotate_shift_geometry) noexcept {
            geometry_type copy{lazy_rotate_shift_geometry};
            copy.rotate_angle() = geometry::TurnRight(copy.rotate_angle());
            copy.shift() = geometry::Add(
                geometry::TurnRight(copy.shift() + copy.geometry_center()),
                -copy.geometry_center());
            return copy;
        }

        template <GeometryConcept T, typename Scalar>
        ALWAYS_INLINE static bool IsApprox(
            const geometry_type& lazy_rotate_shift_geometry,
            T&& any_geometry,
            const Scalar& precision) noexcept {
            return geometry::IsApprox(
                ReverseTransformGeometry(lazy_rotate_shift_geometry, any_geometry),
                lazy_rotate_shift_geometry.wrapped_geometry(), precision);
        }

        template <typename T>
        ALWAYS_INLINE static decltype(auto) Scale(
            const geometry_type& lazy_rotate_shift_geometry, T&& arg) noexcept {
            return geometry::Scale(lazy_rotate_shift_geometry.real_geometry(), arg);
        }

        template <typename T>
        ALWAYS_INLINE static geometry_type Translate(
            const geometry_type& lazy_rotate_shift_geometry, T&& arg) noexcept {
            geometry_type copy{lazy_rotate_shift_geometry};
            copy.shift() = geometry::Add(copy.shift(), arg);
            return copy;
        }

        template <typename Rng, typename Parameters>
        ALWAYS_INLINE PURE static constexpr auto Random(
            Rng& rng, const Parameters& params) noexcept {
            return LazyTransformGeometry{
                geometry::Random<Geometry>(rng, params.geom_params),
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
        ALWAYS_INLINE static decltype(auto) ReverseTransformGeometry(
            const geometry_type& lazy_rotate_shift_geometry,
            const AnyGeometry& any_geometry) noexcept {
            FLOAT_OPTIMIZE

            const auto shifted_geometry_relative_cor = geometry::Translate(
                any_geometry,
                -lazy_rotate_shift_geometry.shift() - lazy_rotate_shift_geometry.geometry_center());
            const auto rotated_shifted_geometry_relative_cor = geometry::RotateCounterClockwise(
                shifted_geometry_relative_cor, -lazy_rotate_shift_geometry.rotate_angle());
            const auto rotated_shifted_geometry = geometry::Translate(
                rotated_shifted_geometry_relative_cor, lazy_rotate_shift_geometry.geometry_center());

            return rotated_shifted_geometry;
        }
    };

    template <IsConcept To, IsLazyTransformGeometry From>
    struct convert_impl<To, From> {
        static constexpr auto Convert(const From& v) {
            return geometry::Convert<To>(v.real_geometry());
        }
    };

} // namespace sdc::geometry
