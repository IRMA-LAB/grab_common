#include <QString>
#include <QtTest>

#include <fstream>
#include <iostream>
#include <string>

#include "kinematics.h"
#include "diffkinematics.h"
#include "types.h"
#include "robotconfigjsonparser.h"

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
  // test parsing with different inputs
  RobotConfigJsonParser parser;
  parser.ParseFile("../test/fail1.json");
  parser.ParseFile(QString("../test/fail2.json"));
  parser.ParseFile(std::string("../test/pass.json"));

  // test getters
  grabcdpr::Params params = parser.GetConfigStruct();
  parser.GetConfigStruct(&params);

  // test display
  parser.PrintConfig();
}

QTEST_APPLESS_MAIN(LibcdprTest)

#include "libcdpr_test.moc"
