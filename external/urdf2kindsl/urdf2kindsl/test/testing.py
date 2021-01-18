import os, io, difflib
import unittest

from urdf2kindsl import urdf, kindsl, convert

thisDir = os.path.dirname(os.path.abspath(__file__))


class MiscTests(unittest.TestCase):
    def test_no_root(self):
        input_urdf = os.path.join(thisDir, 'no_root', 'dummy.urdf')
        urdfin     = urdf.URDFWrapper( input_urdf )
        convert.logger.disabled = True
        self.assertRaises(RuntimeError, convert.Converter, urdfin)
        convert.logger.disabled = False


class CompareExpectedOutputTests(unittest.TestCase):
    defaultNumFormatter = kindsl.NumFormatter()
    differ = difflib.Differ()


    def common(self, input_urdf, expected_kindsl, options={}):
        expected = open(expected_kindsl, 'r')

        generated = io.StringIO()
        urdfin    = urdf.URDFWrapper( input_urdf )
        converted = convert.Converter( urdfin, options )
        writer    = kindsl.Serializer(generated, numFormatter=CompareExpectedOutputTests.defaultNumFormatter)
        writer.writeModel( converted )
        generated.seek(0)

        diff = difflib.unified_diff(expected.readlines(), generated.readlines(), fromfile=expected_kindsl, tofile='<generated>', n=0)
        count = 0
        for line in diff :
            print(line)
            count = count + 1
        if count > 0 :
            # if something is different, save the whole file for the user to ispect
            o = open('test-generated.kindsl','w')
            generated.seek(0)
            o.write(generated.getvalue())
            o.close()
        expected.close()
        generated.close()
        assert(count == 0)


    def test_basic(self):
        subdir = '01'
        input_urdf      = os.path.join(thisDir, subdir, 'ur5.urdf')
        expected_kindsl = os.path.join(thisDir, subdir, 'ur5.kindsl')
        self.common(input_urdf, expected_kindsl)

    def test_keep_dummies(self):
        subdir = '02'
        input_urdf      = os.path.join(thisDir, subdir, 'ur5.urdf')
        expected_kindsl = os.path.join(thisDir, subdir, 'ur5.kindsl')
        options = {}
        options[convert.opt_key_prune] = False
        self.common(input_urdf, expected_kindsl, options)

    def test_prune_dummies(self):
        subdir = '03'
        input_urdf      = os.path.join(thisDir, subdir, 'anymal.urdf')
        expected_kindsl = os.path.join(thisDir, subdir, 'anymal.kindsl')
        options = {}
        options[convert.opt_key_prune] = True
        self.common(input_urdf, expected_kindsl, options)

    def test_prune_fixed_joints(self):
        subdir = 'pruning'
        input_urdf      = os.path.join(thisDir, subdir, 'fixed_joints.urdf')
        expected_kindsl = os.path.join(thisDir, subdir, 'pruned.kindsl')
        options = {
            convert.opt_key_prune    : True,
            convert.opt_key_toframes : True,
            convert.opt_key_lumpi    : True
        }
        self.common(input_urdf, expected_kindsl, options)


if __name__ == "__main__":
    unittest.main()