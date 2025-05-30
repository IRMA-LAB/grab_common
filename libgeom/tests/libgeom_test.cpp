#include <QString>
#include <QtTest>

#include "rotations.h"
#include "quaternions.h"

class LibgeomTest : public QObject
{
  Q_OBJECT

private Q_SLOTS:
  void testCase1();
};

void LibgeomTest::testCase1()
{
  QVERIFY2(true, "Failure");
}

QTEST_APPLESS_MAIN(LibgeomTest)

#include "libgeom_test.moc"
