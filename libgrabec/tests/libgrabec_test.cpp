#include <QString>
#include <QtTest>

#include "types.h"

class LibgrabecTest : public QObject
{
  Q_OBJECT

private Q_SLOTS:
  void testDispRetVal();
};

void LibgrabecTest::testDispRetVal()
{
  grabec::RetVal ret = grabec::OK;
  grabec::DispRetVal(ret, "Displaying value %u. This should be black: ", ret);
  ret = grabec::ECONFIG;
  grabec::DispRetVal(ret, "Displaying value %u. This should be red: ", ret);
}

QTEST_APPLESS_MAIN(LibgrabecTest)

#include "libgrabec_test.moc"
