import math
from urdf2kindsl import numeric

class NumFormatter :
    '''Rounds the floating point numbers for pretty printing
    '''
    def __init__(self, round_digits=6, pi_round_digits=5):
        self.round_decimals = round_digits
        self.pi_round_decimals = pi_round_digits

        self.roundedPI     = round(math.pi,   self.pi_round_decimals)
        self.roundedHalfPI = round(math.pi/2, self.pi_round_decimals)

        self.formatStr = '0:.' + str(self.round_decimals)

    def float2str(self, num, angle=False ) :
        if angle :
            value = round(num, self.pi_round_decimals)
            sign  = "-" if value<0 else ""
            if abs(value) == self.roundedPI :
                return sign+"PI"
            if abs(value) == self.roundedHalfPI :
                return sign+"PI/2.0"

        num = round(num, self.round_decimals)
        num += 0  # this trick avoids the annoying '-0.0' (minus zero)

        # I can't use the '.<n>f' right away because it inserts trailing zeros
        # to fill up <n> decimal positions, which is annoying.
        # So, I first apply standard formatting (no trailing zeros), and then
        # get rid of the scientific notation in case it has been used.
        ret = ( "{" + self.formatStr + "}" ).format( num )
        if "e" in ret:
            ret = ( "{" + self.formatStr + "f}" ).format( num )
        return ret


class Serializer :
    '''Writes the Kinematics-DSL document corresponding to the given Converter instance
    '''
    def __init__(self, outfile, numFormatter=NumFormatter(), floating=False):
        self.floating = floating
        self.__ind = 0
        self.linkID = 1
        self.file = outfile
        self.formatter = numFormatter

    def vec3Str(self, prefix, tupl, angles=False):
        return prefix + '({0[0]:s}, {0[1]:s}, {0[2]:s})'.format(
            [self.formatter.float2str(s, angle=angles) for s in tupl] )

    def indent(self):
        self.__ind += 4
    def indentback(self):
        self.__ind -= 4
    def myprint(self, text):
        line = self.__ind*' ' + text + '\n'
        unicodeline = line + u''   # only way I found to get an unicode for Python 2 AND 3
        self.file.write(unicodeline)

    def _printFrame(self, tr, rot) :
        self.myprint( self.vec3Str('translation = ', tr ) )
        self.myprint( self.vec3Str('rotation    = ', rot, angles=True) )

    def _blockStart(self, name):
        self.myprint(name + ' {')
        self.indent()

    def _blockEnd(self):
        self.indentback()
        self.myprint('}')

    def printJoint(self, j):
        if(j.type == 'prismatic') :
            keyw = 'p_joint'
        else :
            keyw = 'r_joint'
        self._blockStart(keyw + ' ' + j.name)
        self._blockStart('ref_frame')
        self._printFrame( j.frame.tr, j.frame.rot )
        self._blockEnd()
        self._blockEnd()

    def printInertiaParams(self, params):
        self._blockStart('inertia_properties')
        self.myprint('mass = ' + self.formatter.float2str(params['mass']) )
        self.myprint( self.vec3Str('CoM = ', params['com']) )
        for m in ['Ix', 'Iy', 'Iz', 'Ixy', 'Ixz', 'Iyz'] :
            self.myprint(m + (3-len(m))*' ' + '= ' + self.formatter.float2str(params[m]) )
        self._blockEnd()

    def printChildren(self, link):
        self._blockStart('children')
        for child in link.children :
            self.myprint( child[0].name + ' via ' + child[1].name )
        self._blockEnd()

    def printUserFrames(self, link):
        urdfRots  = numeric.getIntrinsicXYZFromR(link.rcg_R_urdf)
        urdfFrame =  any( [math.fabs(x)>1e-5 for x in urdfRots] )
        if urdfFrame or (len(link.frames)>0) :
            self._blockStart('frames')
            for uf in link.frames.keys() :
                self._blockStart(uf)
                self._printFrame(link.frames[uf].tr, link.frames[uf].rot)
                self._blockEnd()

            if urdfFrame :
                self._blockStart('urdf_' + link.name)
                self._printFrame( (0.0,0.0,0.0), urdfRots )
                self._blockEnd()
            self._blockEnd()

    def printLinks_DFS(self, root ) : #DFS = Depth-First-Search
        for child in root.children:
            link = child[0]
            self._blockStart('link ' + link.name)
            self.myprint('id = ' + self.linkID.__str__())
            self.printInertiaParams(link.inertia)
            self.printChildren(link)
            self.printUserFrames(link)
            self._blockEnd()
            self.myprint('\n')

            self.linkID += 1
            self.printLinks_DFS(link)


    def writeModel(self, converted):
        self.myprint('Robot ' + converted.robotName + '\n{\n')
        robotBase = converted.root
        if self.floating:
            self._blockStart('RobotBase ' + robotBase.name + ' floating')
        else:
            self._blockStart('RobotBase ' + robotBase.name)
        self.printInertiaParams(robotBase.inertia)
        self.printChildren(robotBase)
        self.printUserFrames(robotBase)
        self._blockEnd()
        self.myprint('\n')

        self.printLinks_DFS(robotBase)

        for j in converted.joints.values():
            self.printJoint(j)
            self.myprint('')
        self.myprint('}\n')
