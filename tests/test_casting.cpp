#include <gtest/gtest.h>
#include <type_traits>
#include <typeinfo>
#include <cmath>
#include <string>

#include "domain/SimTime.hpp"
#include "domain/BatteryLevel.hpp"
#include "domain/Priority.hpp"
#include "domain/Units.hpp"
#include "domain/Coordinate.hpp"
#include "domain/PlanarPoint.hpp"
#include "domain/Mission.hpp"
#include "domain/Drone.hpp"
#include "events/Event.hpp"
#include "events/DispatchEvent.hpp"
#include "events/FaultEvent.hpp"

using namespace simulator::mesh;

// ============================================================================
// SECTION A & B: Compile-Time Verification of Implicit vs. Explicit Casts
// ============================================================================

// 1. Basic-to-Class Converting Constructors
// Meters from double: IMPLICIT
static_assert(std::is_convertible_v<double, domain::Meters>,
              "Meters should be implicitly constructible from double");

// SimTime from long/int: IMPLICIT
static_assert(std::is_convertible_v<long, domain::SimTime>,
              "SimTime should be implicitly constructible from long");

// Priority from int: EXPLICIT (constructible, but NOT implicitly convertible)
static_assert(!std::is_convertible_v<int, domain::Priority>,
              "Priority must NOT be implicitly convertible from int");
static_assert(std::is_constructible_v<domain::Priority, int>,
              "Priority must be explicitly constructible from int");

// BatteryLevel from float: EXPLICIT (constructible, but NOT implicitly convertible)
static_assert(!std::is_convertible_v<float, domain::BatteryLevel>,
              "BatteryLevel must NOT be implicitly convertible from float");
static_assert(std::is_constructible_v<domain::BatteryLevel, float>,
              "BatteryLevel must be explicitly constructible from float");

// 2. Class-to-Basic Conversion Operators
// SimTime to long: IMPLICIT
static_assert(std::is_convertible_v<domain::SimTime, long>,
              "SimTime should be implicitly convertible to long");

// Meters to double: IMPLICIT
static_assert(std::is_convertible_v<domain::Meters, double>,
              "Meters should be implicitly convertible to double");

// Priority to int: EXPLICIT
static_assert(!std::is_convertible_v<domain::Priority, int>,
              "Priority must NOT be implicitly convertible to int");

// BatteryLevel to float: EXPLICIT
static_assert(!std::is_convertible_v<domain::BatteryLevel, float>,
              "BatteryLevel must NOT be implicitly convertible to float");

// Drone / Mission to std::string: EXPLICIT
static_assert(!std::is_convertible_v<domain::Mission, std::string>,
              "Mission must NOT be implicitly convertible to std::string");
static_assert(!std::is_convertible_v<domain::Drone, std::string>,
              "Drone must NOT be implicitly convertible to std::string");

// ============================================================================
// Runtime Tests: Basic-to-Class & Class-to-Basic
// ============================================================================

TEST(CastingTest, SimTimeImplicitConversions) {
    // Basic-to-Class (Implicit)
    domain::SimTime t1 = 500L;
    domain::SimTime t2 = 1200;
    EXPECT_EQ(t1.getMs(), 500L);
    EXPECT_EQ(t2.getMs(), 1200L);

    // Class-to-Basic (Implicit operator long)
    long scalar = t1;
    EXPECT_EQ(scalar, 500L);

    // Arithmetic and comparison using implicit operator long
    EXPECT_TRUE(t1 < t2);
    EXPECT_EQ(t1 + 500L, 1000L);
    EXPECT_EQ(t2 - t1, 700L);
}

TEST(CastingTest, BatteryLevelExplicitAndClamping) {
    // Basic-to-Class (Explicit required)
    domain::BatteryLevel bNormal(75.5f);
    EXPECT_FLOAT_EQ(bNormal.getPercent(), 75.5f);

    // Clamping to [0, 100]
    domain::BatteryLevel bOver(150.0f);
    EXPECT_FLOAT_EQ(bOver.getPercent(), 100.0f);

    domain::BatteryLevel bUnder(-25.0f);
    EXPECT_FLOAT_EQ(bUnder.getPercent(), 0.0f);

    domain::BatteryLevel bExactZero(0.0f);
    EXPECT_FLOAT_EQ(bExactZero.getPercent(), 0.0f);

    domain::BatteryLevel bExactHundred(100.0f);
    EXPECT_FLOAT_EQ(bExactHundred.getPercent(), 100.0f);

    // Class-to-Basic (Explicit operator float via static_cast)
    float val = static_cast<float>(bNormal);
    EXPECT_FLOAT_EQ(val, 75.5f);
}

TEST(CastingTest, PriorityExplicitAndNamedConstants) {
    // Basic-to-Class (Explicit required)
    domain::Priority pCustom(5);
    EXPECT_EQ(pCustom.getValue(), 5);

    // Named constants
    EXPECT_EQ(static_cast<int>(domain::Priority::LOW), domain::Priority::LOW_VAL);
    EXPECT_EQ(static_cast<int>(domain::Priority::NORMAL), domain::Priority::NORMAL_VAL);
    EXPECT_EQ(static_cast<int>(domain::Priority::HIGH), domain::Priority::HIGH_VAL);
    EXPECT_EQ(static_cast<int>(domain::Priority::CRITICAL), domain::Priority::CRITICAL_VAL);

    // Ordering comparisons
    EXPECT_TRUE(domain::Priority::LOW < domain::Priority::NORMAL);
    EXPECT_TRUE(domain::Priority::NORMAL < domain::Priority::HIGH);
    EXPECT_TRUE(domain::Priority::HIGH < domain::Priority::CRITICAL);

    // Class-to-Basic (Explicit operator int via static_cast)
    int pVal = static_cast<int>(domain::Priority::CRITICAL);
    EXPECT_EQ(pVal, 10);
}

TEST(CastingTest, MetersImplicitConversions) {
    // Basic-to-Class (Implicit)
    domain::Meters m1 = 1500.5;
    EXPECT_DOUBLE_EQ(m1.getValue(), 1500.5);

    // Class-to-Basic (Implicit operator double)
    double d = m1;
    EXPECT_DOUBLE_EQ(d, 1500.5);

    // Use in math expressions
    EXPECT_DOUBLE_EQ(m1 * 2.0, 3001.0);
}

TEST(CastingTest, DroneAndMissionExplicitStringConversion) {
    domain::Coordinate loc(10.0, 20.0);
    domain::Drone drone("DRN-042", loc);
    drone.setBatteryLevel(84.5);

    // Explicit operator std::string() const on Drone
    std::string droneStr = static_cast<std::string>(drone);
    EXPECT_NE(droneStr.find("DRN-042"), std::string::npos);
    EXPECT_NE(droneStr.find("IDLE"), std::string::npos);
    EXPECT_NE(droneStr.find("84.5%"), std::string::npos);

    // Explicit operator std::string() const on Mission
    domain::Mission mission("MSN-100", domain::Mission::PRIORITY_HIGH, loc, loc);
    std::string missionStr = static_cast<std::string>(mission);
    EXPECT_NE(missionStr.find("MSN-100"), std::string::npos);
    EXPECT_NE(missionStr.find("HIGH"), std::string::npos);
    EXPECT_NE(missionStr.find("PENDING"), std::string::npos);
}

// ============================================================================
// SECTION C: Class-to-Class Casting Tests
// ============================================================================

TEST(CastingTest, CoordinateToPlanarPointRoundTrip) {
    domain::Coordinate coord(123.456, 789.012);

    // Direction 1: Converting constructor Coordinate -> PlanarPoint
    domain::PlanarPoint pt(coord);
    EXPECT_DOUBLE_EQ(pt.getX(), 123.456);
    EXPECT_DOUBLE_EQ(pt.getY(), 789.012);

    // Direction 2: Conversion operator PlanarPoint -> Coordinate
    domain::Coordinate roundTrip = pt;
    EXPECT_NEAR(roundTrip.getLatitude(), coord.getLatitude(), 1e-9);
    EXPECT_NEAR(roundTrip.getLongitude(), coord.getLongitude(), 1e-9);

    // Distance calculation on PlanarPoint
    domain::PlanarPoint pt2(126.456, 793.012);
    // dx = 3, dy = 4 -> dist = 5
    EXPECT_NEAR(pt.distanceTo(pt2), 5.0, 1e-9);
}

// ============================================================================
// Polymorphic Event Casting Tests (dynamic_cast, static_cast, std::bad_cast)
// ============================================================================

TEST(CastingTest, DynamicCastPointerSuccessAndFailure) {
    domain::Mission m("MSN-001");
    events::DispatchEvent dispatchEv(1000, 3, &m);
    events::FaultEvent faultEv(2000, 10, 404, nullptr);

    events::Event* ev1 = &dispatchEv;
    events::Event* ev2 = &faultEv;

    // dynamic_cast success: DispatchEvent* from Event*
    events::DispatchEvent* pDispatch = dynamic_cast<events::DispatchEvent*>(ev1);
    ASSERT_NE(pDispatch, nullptr);
    EXPECT_EQ(pDispatch->getTargetMission()->getMissionId(), "MSN-001");

    // dynamic_cast failure: returns nullptr when type does not match
    events::FaultEvent* pNotFault = dynamic_cast<events::FaultEvent*>(ev1);
    EXPECT_EQ(pNotFault, nullptr);

    events::DispatchEvent* pNotDispatch = dynamic_cast<events::DispatchEvent*>(ev2);
    EXPECT_EQ(pNotDispatch, nullptr);

    // dynamic_cast success for FaultEvent
    events::FaultEvent* pFault = dynamic_cast<events::FaultEvent*>(ev2);
    ASSERT_NE(pFault, nullptr);
    EXPECT_EQ(pFault->getErrorCode(), 404);
}

TEST(CastingTest, DynamicCastReferenceWithBadCastException) {
    domain::Mission m("MSN-002");
    events::DispatchEvent dispatchEv(1000, 3, &m);
    events::FaultEvent faultEv(2000, 10, 500, nullptr);

    events::Event& evRef1 = dispatchEv;
    events::Event& evRef2 = faultEv;

    // Successful reference dynamic_cast
    EXPECT_NO_THROW({
        const auto& dRef = dynamic_cast<const events::DispatchEvent&>(evRef1);
        EXPECT_EQ(dRef.getTargetMission()->getMissionId(), "MSN-002");
    });

    // Failing reference dynamic_cast throws std::bad_cast
    EXPECT_THROW({
        (void)dynamic_cast<const events::FaultEvent&>(evRef1);
    }, std::bad_cast);

    EXPECT_THROW({
        (void)dynamic_cast<const events::DispatchEvent&>(evRef2);
    }, std::bad_cast);
}

TEST(CastingTest, StaticCastGuaranteedType) {
    domain::Mission m("MSN-003");
    events::DispatchEvent dispatchEv(1000, 3, &m);
    events::Event* base = &dispatchEv;

    // static_cast downcast is safe only after the type has been verified
    ASSERT_STREQ(base->getEventType(), "DispatchEvent");
    // static_cast avoids runtime RTTI overhead
    auto* concrete = static_cast<events::DispatchEvent*>(base);
    EXPECT_EQ(concrete->getTargetMission()->getMissionId(), "MSN-003");
}
