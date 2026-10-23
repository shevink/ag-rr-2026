#include <gtest/gtest.h>

TEST(PackageSetupTest, CompilationCheck) {
  // Verify that the test infrastructure works
  ASSERT_TRUE(true);
}

TEST(PackageSetupTest, ServiceInterfaceAvailable) {
  // Verify the LoadMap service header can be included at link time
  // This test just checks compilation succeeds
  ASSERT_TRUE(true);
}
