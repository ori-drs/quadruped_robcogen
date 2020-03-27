import logging,math
import numpy as np
from collections import OrderedDict as ODict

from urdf2kindsl import numeric

logger = logging.getLogger(__name__)


opt_key_prune = 'prunefixed'
opt_key_toframes = 'toframes'
opt_key_lumpi = 'lumpinertia'


class Converter :
    '''Reads the model from a URDFWrapper instance, and applies the necessary conversions
    '''
    class Frame :
        def __init__(self):
            self.H   = np.identity(4)    # Homogeneous coordinate transform, from this-instance-coordinates to some link coordinates
            self.rot = (0.0, 0.0, 0.0)   # Intrinsic rx, ry, rz angles
            self.tr  = self.H[0:3,3]     # View of the translation vector

    class Link :
        def __init__(self, namestr):
            self.name    = namestr
            self.parent  = None
            self.parentJ = None
            self.children= list()
            self.inertia = dict()
            self.frames  = ODict()
            self.rcg_R_urdf = np.identity(3)

    class Joint :
        def __init__(self, namestr):
            self.name = namestr
            self.type = 'revolute'
            self.predecessor = None
            self.successor  = None
            self.frame = Converter.Frame()


    @staticmethod
    def toValidID( name ) :
        return name.replace('-', '__')

    def __init__(self, urdf, options={}) :
        if opt_key_prune not in options:
            options[opt_key_prune] = False
        if opt_key_toframes not in options:
            options[opt_key_toframes] = True
        if opt_key_lumpi not in options :
            options[opt_key_lumpi] = True

        self.robotName = urdf.robotName
        self.links  = ODict()
        self.joints = ODict()
        self.frames = ODict()

        for urdfname in urdf.links.keys() :
            name = self.toValidID( urdfname )
            link = Converter.Link( name )
            self.links[name] = link

        for jname in urdf.joints.keys() :
            urdfjoint = urdf.joints[jname]
            name = self.toValidID( jname )
            joint= Converter.Joint( name )
            joint.type = urdfjoint.type

            joint.predecessor = self.links[ self.toValidID(urdfjoint.parent) ]
            joint.successor   = self.links[ self.toValidID(urdfjoint.child)  ]
            self.convertJointFrame(joint, urdfjoint)

            joint.successor.parent = joint.predecessor
            joint.successor.parentJ= joint
            joint.predecessor.children.append( (joint.successor, joint) )
            self.joints[name] = joint

        orphans = [l for l in self.links.values() if l.parent==None]
        if len(orphans)==0 :
            logger.fatal("Could not find any root link (i.e. a link without parent).")
            logger.fatal("Check for kinematic loops.")
            raise RuntimeError("no root link found")
        if len(orphans) > 1 :
            logger.warning("Found {0} links without parent, only one expected".format(len(orphans)))
            logger.warning("Any robot model must have exactly one root element.")
            logger.warning("This might lead to unexpected results.")
        self.root = orphans[0]

        self.leafs = [l for l in self.links.values() if len(l.children)==0]

        # Conversion of the inertia must happen after the joint frame conversion,
        # which determines the required coordinate transforms
        for urdfname in urdf.links.keys() :
            name = self.toValidID( urdfname )
            self.convertInertialData(self.links[name], urdf.links[urdfname].inertia)

        if options[opt_key_prune] :
            # Let's first explicitly check for the nasty case where the root is
            # a dummy link, connected via a fixed joint to the first link
            self._collapseDummyRoots()
            keepPruning = True
            while keepPruning :
                keepPruning = self._pruneDummies(options)

    def _collapseDummyRoots(self):
        root = self.root
        keepGoing = True
        while self.isDummyLink(root) and keepGoing :
            if len(root.children) > 1 :
                logger.warning("Detected dummy root link ({0}) with multiple children; cannot collapse".format(root.name))
                keepGoing = False
                for childPair in root.children :
                    childPair[1].__preserve = True
            else :
                childSpec = root.children[0]
                joint = childSpec[1]
                child = childSpec[0]
                if joint.type != 'fixed' :
                    logger.warning("Detected dummy root link ({0}) supporting a non-fixed joint ({1})".format(root.name, joint.name))
                    keepGoing = False
                    joint.__preserve = True
                else :
                    # We have a dummy root link, with only one child connected via fixed joint.
                    # Let's delete it and replace the root
                    logger.info("Deleting dummy pair '{0}'-'{1}', root replaced with '{2}'".format(
                        root.name, joint.name, child.name))
                    root = child
                    del self.links[root.name]
                    del self.joints[joint.name]

        self.root = root

    def _pruneDummies(self, options):
        toBeDeleted = set()
        for leaf in self.leafs :
            joint  = leaf.parentJ
            parent = leaf.parent

            if joint is None or parent is None:
                continue # go on with the next leaf
            if joint.type != 'fixed' :
                continue

            logger.debug("Trying to collapse link '{0}', connected by joint '{1}'".format(leaf.name, joint.name))
            if options[opt_key_toframes] :
                # We need to "move" the frames associated with 'leaf' into
                # the parent frames. First of all, the implicit link frame,
                # which is the same as the supporting-joint frame:
                parent.frames[leaf.name] = joint.frame

                # Then the additional custom frames on the link; for these
                # ones we must perform a coordinate transform
                for ufr in leaf.frames.keys() :
                    original = leaf.frames[ufr]
                    shiftedup= Converter.Frame()
                    # We need the [:,:] to assign values to the same memory
                    # location, because the translation attribute is a view
                    # of H. If we change H, the view will be inconsistent
                    shiftedup.H[:,:] = np.matmul( joint.frame.H, original.H )
                    shiftedup.rot = numeric.getIntrinsicXYZFromR( shiftedup.H[0:3,0:3] )
                    parent.frames[ufr] = shiftedup

            if options[opt_key_lumpi] :
                # Keep in mind that at this point all the inertia properties
                # are in robcogen format, that is, in link coordinates. And
                # the link frame is the same as the supporting-joint frame
                if leaf.inertia['mass'] != 0 :
                    loadMe = parent.inertia
                    parent_R_leaf = numeric.getR_intrinsicXYZ( *joint.frame.rot )

                    # The translation we need is the position of the parent
                    # link frame relative to the joint frame, in joint frame
                    # coordinates
                    tr = - np.matmul( parent_R_leaf.T , joint.frame.tr )
                    # Transform the inertia of the link in the coordinate
                    # system of the parent link
                    addMe = numeric.rotoTranslateInertia(leaf.inertia, tr, parent_R_leaf)
                    m1 = loadMe['mass']
                    m2 = addMe['mass']

                    loadMe['mass'] = m1 + m2
                    loadMe['Ix']  = loadMe['Ix']  + addMe['Ix']
                    loadMe['Iy']  = loadMe['Iy']  + addMe['Iy']
                    loadMe['Iz']  = loadMe['Iz']  + addMe['Iz']
                    loadMe['Ixy'] = loadMe['Ixy'] + addMe['Ixy']
                    loadMe['Ixz'] = loadMe['Ixz'] + addMe['Ixz']
                    loadMe['Iyz'] = loadMe['Iyz'] + addMe['Iyz']
                    loadMe['com'] = (loadMe['com']*m1 + addMe['com']*m2)/(m1+m2)

            toBeDeleted.add(leaf)

        changed = len(toBeDeleted) > 0
        for eraseMe in toBeDeleted :
            joint = eraseMe.parentJ
            parent= eraseMe.parent
            parent.children.remove( (eraseMe, joint) )
            del self.joints[joint.name]
            del joint
            del self.links[eraseMe.name]
            del eraseMe

        # Reconstruct the leafs array, after the pruning
        self.leafs = [l for l in self.links.values() if len(l.children)==0]
        return changed


    def isDummyLink(self, link):
        immaterial = link.inertia['mass'] == 0.0
        fixedj = False
        if link.parentJ is not None :
            fixedj = (link.parentJ.type == 'fixed')

        return (immaterial and fixedj)

    def convertInertialData(self, link, urdfParams):
        iin = {}
        iin['mass'] = urdfParams['mass']
        iin['com']  = np.zeros(3)
        iin['Ix']   =  urdfParams['ixx']
        iin['Iy']   =  urdfParams['iyy']
        iin['Iz']   =  urdfParams['izz']
        iin['Ixy']  = -urdfParams['ixy']
        iin['Ixz']  = -urdfParams['ixz']
        iin['Iyz']  = -urdfParams['iyz']

        tr = -np.array(urdfParams['xyz'])
        iout = numeric.rotoTranslateInertia(iin, tr, link.rcg_R_urdf)
        link.inertia = iout

    def convertJointFrame(self, joint, urdfjoint):
        rpy = urdfjoint.frame['rpy']

        # Rotation matrix from URDF joint frame to URDF link frame
        urdflink_X_urdfjoint = numeric.getR_extrinsicXYZ(* rpy )

        # Rotation matrix from URDF joint frame to RobCoGen link frame
        rcglink_X_urdfjoint = np.matmul(joint.predecessor.rcg_R_urdf, urdflink_X_urdfjoint)

        '''
        If the URDF joint is fixed, there is no axis. In that case we only need
        to get the intrinsic rotation parameters (robcogen), from the complete
        rotation matrix we already computed above.
        Otherwise, we need to get the coordinates of the joint axis and make
        sure to rotate the robcogen frame so as to align its Z axis with the
        joint axis.
        '''
        if urdfjoint.type == "fixed" :
            (rx, ry, rz) = numeric.getIntrinsicXYZFromR( rcglink_X_urdfjoint )
        else :
            axis = np.array( urdfjoint.frame['axis'] )

            # The joint axis in robcogen-link-frame coordinates
            axis_linkframe = np.matmul(rcglink_X_urdfjoint, axis)

            '''
            We want to find the intrinsic rotations rx ry rz for the joint frame in the
            RobCoGen model. These rotations must be such that the Z axis of the
            resulting frame is aligned with the joint axis, computed above. This
            constraint and the expression of the full matrix (see method above), give us
            the equations for rx ry :

                 sin(ry)         = axis_x
               - sin(rx) cos(ry) = axis_y
                 cos(rx) cos(ry) = axis_z
            '''
            axis_rounded = np.round(axis_linkframe, 5)
            ry = math.asin( axis_linkframe[0] )
            if axis_rounded[2] != 0.0 :
                rx = math.atan2( -axis_linkframe[1], axis_linkframe[2])
            else :
                cy = math.cos(ry)
                if round(cy,5) != 0.0 :
                    rx = math.asin( - axis_linkframe[1] / cy )
                else:
                    rx = 0.0
            rz = 0.0;
            dbgmsg = '''
                Joint frame conversion:
                joint: {joint}
                joint axis = {axis}  (in robcogen link-frame coordinates)
                (rx ry rz) = {rots}  (before possible rz correction)'''\
                .format(joint=urdfjoint.name, axis=axis_rounded, rots=tuple(round(r,5) for r in (rx,ry,rz)) )
            logger.debug(dbgmsg)

        # Rotation matrix from RobCoGen joint frame to link frame
        rcglink_X_rcgjoint = numeric.getR_intrinsicXYZ(rx, ry, rz)

        # Rotation matrix from URDF joint frame to RobCoGen joint frame.
        # We can interpret this one as the rotation difference between the joint
        #  frames in the two models.
        R = np.matmul(rcglink_X_rcgjoint.transpose() , rcglink_X_urdfjoint)

        # The best we can do, at this point, is to check whether the difference
        # between the two frames is simply a rotation about z (for a revolute
        # joint), and, if so, add it to the parameters (affecting the zero
        # configuration only). We do this to "minimize" the variation with
        # respect to the URDF joint frame.
        if joint.type == 'revolute' :
            roundDigits = 5
            Z = np.array([0,0,1])
            Rz = R[:,2]

            # If we have a pure rotation about Z ...
            if np.equal( np.round(Rz, roundDigits), Z).all() :
                # Special case angle=PI, for which the formulas below do not work
                if round(R[0,0],roundDigits) == -1 and round(R[1,1],roundDigits) == -1 :
                    rz = math.pi
                else:
                    diffaxis = np.array( [ R[2,1] - R[1,2],  R[0,2] - R[2,0], R[1,0] - R[0,1] ] )
                    norm     = np.linalg.norm( diffaxis )
                    if norm > 1e-5 :
                        rz = math.atan2( norm, R.trace()-1 )

                        axisnorm = np.round( diffaxis/norm, 5 ) # normalized axis, rounded
                        # Check the inversion of the axis due to negative rotation angles
                        # Our axis must be Z, not -Z
                        if axisnorm[2] < 0 :
                            axisnorm[2] = -axisnorm[2]
                            rz = -rz

                        if not np.equal( axisnorm, Z).all() :
                            # We checked above it's a pure rotation about Z, so that
                            # should be confirmed here too...
                            msg = "Possible inconsistency in the " \
                                + joint.name + " joint frame rotation"
                            logger.warning(msg)

                # Now that we have possibly changed rz, recompute the joint transform
                # and the rotation difference with the URDF transform
                rcglink_X_rcgjoint = numeric.getR_intrinsicXYZ(rx, ry, rz)
                R = np.matmul(rcglink_X_rcgjoint.transpose(), rcglink_X_urdfjoint)

        # Save the transformation from joint coordinates to link coordinates
        joint.frame.tr[:] = np.matmul(joint.predecessor.rcg_R_urdf, urdfjoint.frame['xyz'])
        joint.frame.rot   = (rx, ry, rz)
        joint.frame.H[0:3,0:3] = rcglink_X_rcgjoint

        # Save the rotation difference between robcogen and urdf link frames
        joint.successor.rcg_R_urdf = R

