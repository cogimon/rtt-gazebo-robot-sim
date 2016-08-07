#include <gtest/gtest.h>
#include <parser.h>

namespace {
    class test_parser: public ::testing::Test {
    public:
        test_parser(){
        }

        virtual ~test_parser() {

        }

        virtual void SetUp() {

        }

        virtual void TearDown() {

        }

        cogimon::gain_parser gains;
    };

    TEST_F(test_parser, test_parser_)
    {
        std::string file_name = "coman.srdf";
        EXPECT_TRUE(this->gains.initFile(file_name));

        this->gains.printGains();
    }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
