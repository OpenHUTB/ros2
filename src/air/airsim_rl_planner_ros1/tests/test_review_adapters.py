"""ROS-free image transport, model-path and goal-stop regressions."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from airsim_rl_planner_ros1.image_codec import ImageCodec, encode_image
from airsim_rl_planner_ros1.core import model_path, goal_reached

class ReviewAdapters(unittest.TestCase):
    def message(self, array, encoding):
        return SimpleNamespace(**encode_image(array, encoding))

    def test_rgb_to_bgr(self):
        rgb = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
        np.testing.assert_array_equal(ImageCodec().to_array(self.message(rgb, 'rgb8'), 'bgr8'), rgb[:, :, ::-1])

    def test_bgr_roundtrip_noncontiguous(self):
        array = np.arange(36, dtype=np.uint8).reshape(3, 4, 3)[:, ::2]
        msg = self.message(array, 'bgr8')
        self.assertEqual(msg.step, 6)
        np.testing.assert_array_equal(ImageCodec().to_array(msg, 'bgr8'), array)

    def test_color_row_padding(self):
        msg = SimpleNamespace(height=2,width=1,step=4,encoding='rgb8',is_bigendian=0,
                              data=bytes([1,2,3,99,4,5,6,99]))
        np.testing.assert_array_equal(ImageCodec().to_array(msg,'rgb8'), [[[1,2,3]],[[4,5,6]]])

    def test_depth_roundtrip(self):
        array = np.array([[1.25, np.inf], [np.nan, 3.5]], dtype=np.float32)
        msg = self.message(array, '32FC1')
        self.assertEqual(msg.is_bigendian, 0)
        self.assertEqual(msg.step, 8)
        np.testing.assert_allclose(ImageCodec().to_array(msg,'32FC1'), array)

    def test_depth_big_endian_padded(self):
        data = np.array([1.25],dtype='>f4').tobytes()+b'xxxx'+np.array([2.5],dtype='>f4').tobytes()+b'xxxx'
        msg = SimpleNamespace(height=2,width=1,step=8,encoding='32FC1',is_bigendian=1,data=data)
        np.testing.assert_array_equal(ImageCodec().to_array(msg,'32FC1'),[[1.25],[2.5]])

    def test_bad_length_step_and_empty(self):
        for h,w,step,data in [(1,2,3,b'123'),(2,1,3,b'123'),(0,1,3,b'')]:
            msg=SimpleNamespace(height=h,width=w,step=step,data=data,encoding='rgb8',is_bigendian=0)
            with self.assertRaises(ValueError): ImageCodec().to_array(msg,'rgb8')

    def test_reject_incompatible_encodings(self):
        msg=self.message(np.ones((1,1),np.float32),'32FC1')
        with self.assertRaises(ValueError): ImageCodec().to_array(msg,'bgr8')
        msg.encoding='mono16'
        with self.assertRaises(ValueError): ImageCodec().to_array(msg,'32FC1')

    def test_reject_invalid_arrays(self):
        for array,encoding in [(np.zeros((2,2)), 'rgb8'),(np.zeros((2,2,3)), 'rgb8'),(np.zeros((0,2)), '32FC1'),(np.zeros((2,2,3)), '32FC1')]:
            with self.assertRaises(ValueError): encode_image(array,encoding)

    def test_empty_model_names_parameter(self):
        for name in ['policy_path','yolo_path']:
            for value in ['', ' ', None]:
                with self.assertRaisesRegex(ValueError, name): model_path(value,name)

    def test_missing_model_names_parameter(self):
        with self.assertRaisesRegex(FileNotFoundError,'policy_path file not found'):
            model_path(str(Path(__file__).parent/'not_a_model.zip'),'policy_path')

    def test_model_is_resolved_file(self):
        self.assertEqual(model_path(__file__,'policy_path'), Path(__file__).resolve())
        with self.assertRaises(FileNotFoundError): model_path(str(Path(__file__).parent),'yolo_path')

    def test_goal_stop_requires_opt_in(self):
        self.assertFalse(goal_reached(np.zeros(3), [0,0,0], 2))
        self.assertTrue(goal_reached(np.zeros(3), [0,0,0], 2, True))
        self.assertFalse(goal_reached(np.zeros(3), [3,0,0], 2, True))

if __name__ == '__main__': unittest.main()
