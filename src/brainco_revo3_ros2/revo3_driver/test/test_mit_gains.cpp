#include <limits>

#include "gtest/gtest.h"
#include "revo3_driver/mit_gains.hpp"

TEST(MitGains, PreservesExplicitReleaseAndActiveGains)
{
  for (double fallback : {revo3_driver::kFallbackKp, revo3_driver::kFallbackKd}) {
    EXPECT_DOUBLE_EQ(revo3_driver::mit_gain_or_default(0.0, fallback), 0.0);
    EXPECT_DOUBLE_EQ(revo3_driver::mit_gain_or_default(5.0, fallback), 5.0);
  }
}

TEST(MitGains, InvalidValuesUsePositionControlDefaults)
{
  for (double fallback : {revo3_driver::kFallbackKp, revo3_driver::kFallbackKd}) {
    for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()}) {
      EXPECT_DOUBLE_EQ(revo3_driver::mit_gain_or_default(invalid, fallback), fallback);
    }
  }
}
