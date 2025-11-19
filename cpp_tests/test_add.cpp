#include <gtest/gtest.h>
#include <ateam_controls/ateam_controls.h>

TEST(AteamControls, Add) {
  EXPECT_EQ(ateam_controls_add(10,20), 30);
}
