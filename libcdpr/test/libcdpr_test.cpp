#include <QString>
#include <QtTest>

#include <fstream>
#include <iostream>
#include <string>

#include "kinematics.h"
#include "diffkinematics.h"
#include "json.hpp"

// Aliases ---------------------------------------------------------------------
using json = nlohmann::json; // JSON library support

/**
 * @brief The LibcdprTest class
 */
class LibcdprTest : public QObject
{
  Q_OBJECT

private Q_SLOTS:
  /**
   * @brief testCase1
   */
  void testJsonParser();
};

void LibcdprTest::testJsonParser()
{
  std::ifstream ifile("test.json");

  QVERIFY2(ifile.is_open(), "Could not open input file");

  json robot_config;
  ifile >> robot_config;

  ifile.close();
}

QTEST_APPLESS_MAIN(LibcdprTest)

#include "libcdpr_test.moc"
