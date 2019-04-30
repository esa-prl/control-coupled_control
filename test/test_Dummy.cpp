#include <boost/test/unit_test.hpp>
#include <coupled_control/Dummy.hpp>

using namespace coupled_control;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    coupled_control::DummyClass dummy;
    dummy.welcome();
}
