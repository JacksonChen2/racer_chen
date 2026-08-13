#include <racer_sionna_comm/link_model.hpp>

#include <gtest/gtest.h>

TEST(LinkModel, StrongerSnrImprovesRateAndReliability) {
  racer_sionna_comm::LinkModel model({20.0e6, 0.55, 7.0, 1.8, 1200});
  EXPECT_GT(model.bitRate(23.0), model.bitRate(0.0));
  EXPECT_LT(model.packetErrorRate(23.0, 1200),
            model.packetErrorRate(0.0, 1200));
}

TEST(LinkModel, LargerMessagesHaveMoreErrorsAndSerializationDelay) {
  racer_sionna_comm::LinkModel model({20.0e6, 0.55, 7.0, 1.8, 1200});
  EXPECT_GT(model.packetErrorRate(8.0, 4800),
            model.packetErrorRate(8.0, 600));
  EXPECT_GT(model.serializationDelay(8.0, 4800),
            model.serializationDelay(8.0, 600));
}

TEST(LinkModel, RejectsInvalidConfiguration) {
  EXPECT_THROW(
      racer_sionna_comm::LinkModel({0.0, 0.55, 7.0, 1.8, 1200}),
      std::invalid_argument);
}
