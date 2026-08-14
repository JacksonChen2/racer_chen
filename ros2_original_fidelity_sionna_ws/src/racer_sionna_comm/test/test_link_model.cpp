#include <racer_sionna_comm/link_model.hpp>

#include <gtest/gtest.h>

TEST(LinkModel, StrongerSnrImprovesRateAndReliability) {
  racer_sionna_comm::LinkModel model(
      {100.0e6, 120.0e3, 66, 0.82, 0.10, 1.35, 1200});
  EXPECT_GT(model.bitRate(23.0), model.bitRate(0.0));
  EXPECT_LT(model.packetErrorRate(23.0, 1200),
            model.packetErrorRate(0.0, 1200));
}

TEST(LinkModel, LargerMessagesHaveMoreErrorsAndSerializationDelay) {
  racer_sionna_comm::LinkModel model(
      {100.0e6, 120.0e3, 66, 0.82, 0.10, 1.35, 1200});
  EXPECT_GT(model.packetErrorRate(8.0, 4800),
            model.packetErrorRate(8.0, 600));
  EXPECT_GT(model.serializationDelay(8.0, 4800),
            model.serializationDelay(8.0, 600));
}

TEST(LinkModel, UsesOnlyRequestedAdaptiveModulations) {
  racer_sionna_comm::LinkModel model(
      {100.0e6, 120.0e3, 66, 0.82, 0.10, 1.35, 1200});
  EXPECT_EQ(model.selectMcs(-5.0).name, "QPSK");
  EXPECT_EQ(model.selectMcs(7.0).name, "16QAM");
  EXPECT_EQ(model.selectMcs(14.0).name, "64QAM");
  EXPECT_EQ(model.selectMcs(21.0).name, "256QAM");
  EXPECT_NEAR(model.transportBlockErrorRate(14.0), 0.10, 1.0e-12);
  EXPECT_NEAR(model.slotDuration(), 0.000125, 1.0e-12);
}

TEST(LinkModel, RejectsInvalidConfiguration) {
  EXPECT_THROW(
      racer_sionna_comm::LinkModel(
          {0.0, 120.0e3, 66, 0.82, 0.10, 1.35, 1200}),
      std::invalid_argument);
}
