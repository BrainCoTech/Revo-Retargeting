#include <gtest/gtest.h>
#include <limits>
#include "revo2_driver/normalized_motor_limits.hpp"
#include "revo2_driver/transport_session_base.hpp"

using revo2_driver::NormalizedMotorLimits;

namespace
{
NormalizedMotorLimits sample()
{
  return {{0, 0, 5, 0, 0, 0}, {59, 90, 81, 81, 81, 81}, {145, 160, 130, 130, 130, 130}};
}
int read_number = 0;
int fail_read = -1;
bool physical = false;
bool change_mode = false;
bool invalid_limits = false;

class RegisterSession : public revo2_driver::SessionBase
{
public:
  explicit RegisterSession(revo2_driver::BraincoHandApi::DriverConfig & config)
  : SessionBase(config) {set_handler(reinterpret_cast<DeviceHandler *>(1));}
  bool open() override {return true;}
  void close() override {}
  bool is_open() const override {return true;}
  std::optional<revo2_driver::BraincoHandApi::ConnectionInfo> connection_info() const override
  {return std::nullopt;}
};
}  // namespace

// Interpose only the read-only SDK entry point exercised by SessionBase.
// No transport connection or physical device is opened by these tests.
extern "C" int32_t stark_read_holding_registers(
  DeviceHandler *, uint8_t, uint16_t address, uint16_t count, uint16_t * data)
{
  ++read_number;
  if (read_number == fail_read) {return -1;}
  if (address == 937 && count == 1) {
    data[0] = physical || (change_mode && read_number == 3) ? 1 : 0;
    return 0;
  }
  if (address == 946 && count == 18) {
    auto limits = sample();
    for (std::size_t i = 0; i < 6; ++i) {
      data[i] = limits.min_deg[i];
      data[i + 6] = invalid_limits ? 0 : limits.max_deg[i];
      data[i + 12] = limits.max_speed_deg_s[i];
    }
    return 0;
  }
  return -1;
}

TEST(NormalizedPosition, PerMotorRangesAndNonzeroMinimum)
{
  const auto limits = sample();
  ASSERT_TRUE(limits.valid());
  EXPECT_NEAR(limits.position_rad(0, 1000), 59 * limits.deg_to_rad, 1e-12);
  EXPECT_NEAR(limits.position_rad(1, 1000), 90 * limits.deg_to_rad, 1e-12);
  EXPECT_NEAR(limits.position_rad(2, 0), 5 * limits.deg_to_rad, 1e-12);
  EXPECT_NEAR(limits.position_command(0, 29.5 * limits.deg_to_rad), 500, 1e-10);
  EXPECT_NEAR(limits.position_command(1, 45 * limits.deg_to_rad), 500, 1e-10);
  EXPECT_NEAR(limits.position_command(2, 43 * limits.deg_to_rad), 500, 1e-10);
  for (std::size_t i = 0; i < 6; ++i) {
    for (int raw : {0, 1, 250, 500, 999, 1000}) {
      EXPECT_NEAR(limits.position_command(i, limits.position_rad(i, raw)), raw, 1e-9);
    }
  }
  EXPECT_NEAR(limits.velocity_rad_s(1, -500), -80 * limits.deg_to_rad, 1e-12);
}

TEST(NormalizedPosition, ClampTargetsRejectInvalidInput)
{
  auto limits = sample();
  EXPECT_EQ(limits.position_command(0, -1), 0);
  EXPECT_EQ(limits.position_command(0, 3), 1000);
  EXPECT_THROW(limits.position_command(0, std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(limits.position_rad(0, 1001), std::invalid_argument);
  limits.max_deg[0] = limits.min_deg[0];
  EXPECT_FALSE(limits.valid());
  limits = sample();
  limits.max_speed_deg_s[5] = 0;
  EXPECT_FALSE(limits.valid());
}

TEST(NormalizedPosition, ReadsFailClosedAndChecksModeTwice)
{
  revo2_driver::BraincoHandApi::DriverConfig config;
  RegisterSession session(config);
  for (int failed : {-1, 1, 2, 3}) {
    read_number = 0;
    fail_read = failed;
    const auto limits = session.get_normalized_motor_limits(126);
    EXPECT_EQ(limits.has_value(), failed == -1);
    if (limits) {EXPECT_EQ(limits->max_deg[1], 90);}
  }
  fail_read = -1;
  read_number = 0;
  physical = true;
  EXPECT_FALSE(session.get_normalized_motor_limits(126));
  physical = false;
  read_number = 0;
  change_mode = true;
  EXPECT_FALSE(session.get_normalized_motor_limits(126));
  change_mode = false;
  read_number = 0;
  invalid_limits = true;
  EXPECT_FALSE(session.get_normalized_motor_limits(126));
  invalid_limits = false;
  config.protocol = revo2_driver::Protocol::kCanfd;
  EXPECT_FALSE(session.get_normalized_motor_limits(126));
}
