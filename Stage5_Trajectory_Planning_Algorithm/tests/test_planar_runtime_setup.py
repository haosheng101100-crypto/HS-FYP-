import unittest
from tools.planar_runtime_setup import desktop_environment,prepare_world_start


class RuntimeSetupTests(unittest.TestCase):
    def test_desktop_values_are_inherited_from_current_environment(self):
        actual={'DISPLAY':':8','XAUTHORITY':'/tmp/current-auth','XDG_RUNTIME_DIR':'/run/user/1001','DBUS_SESSION_BUS_ADDRESS':'unix:path=/tmp/current-bus'}
        self.assertEqual(desktop_environment(actual),actual)

    def test_only_known_archived_world_initial_positions_are_accepted(self):
        old='<pose>5 0 0.002 0 0 0</pose>';expected='<pose>3 0 0.002 0 0 0</pose>'
        self.assertEqual(prepare_world_start(old),expected)
        self.assertEqual(prepare_world_start(expected),expected)
        with self.assertRaises(ValueError):prepare_world_start('<pose>99 0 0 0 0 0</pose>')

if __name__=='__main__':unittest.main()
