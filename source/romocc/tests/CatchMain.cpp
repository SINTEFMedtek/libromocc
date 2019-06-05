//
// Created by androst on 05.06.19.
//

#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

int main(int argc, char* argv[]) {
    int result = Catch::Session().run(argc, argv);
    return result;
}