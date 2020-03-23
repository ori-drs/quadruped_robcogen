import logging
import numpy as np
import xml.etree.ElementTree as ET
from collections import OrderedDict as ODict

from urdf2kindsl import numeric

logger = logging.getLogger(__name__)

class URDFWrapper :
    '''Representation of the original links and joints data, as found in a XML URDF
    '''
    class Link:
        def __init__(self, name):
            self.name    = name
            self.inertia = None
            self.parent  = None
            self.supportingJoint = None
    class Joint:
        def __init__(self, name):
            self.name = name
            self.type  = None
            self.frame = None
            self.parent= None
            self.child = None
            self.predec_H_joint = np.identity(4)

    iMomentsLabels = ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']

    def __init__(self, urdfInFile):
        root = ET.parse(urdfInFile)
        self.robotName = root.getroot().get('name')

        linkNodes  = root.findall("link")
        jointNodes = root.findall("joint")

        self.links  = ODict()
        self.joints = ODict()
        self.frames = ODict()

        for nodelink in linkNodes:
            name = nodelink.get('name')
            link = URDFWrapper.Link( name )
            link.inertia = self.readInertialData(nodelink)
            self.links[name] = link

        for nodejoint in jointNodes:
            name = nodejoint.get('name')
            joint = URDFWrapper.Joint( name )
            joint.type  = nodejoint.get('type')
            joint.frame = self.readJointFrameData( nodejoint )
            joint.predec_H_joint[:3,:3] = numeric.getR_extrinsicXYZ( * joint.frame['rpy'] )
            joint.predec_H_joint[:3,3]  = np.array( joint.frame['xyz'] )
            joint.parent= nodejoint.find('parent').get('link')
            joint.child = nodejoint.find('child').get('link')

            # Note I keep URDF nomenclature ("parent" and "child") just to
            # stress the bond with the source URDF XML file. I will later use
            # the more appropriate terms (e.g. "predecessor")

            self.joints[name] = joint

            predecessor = self.links[ joint.parent ]
            successor   = self.links[ joint.child ]
            successor.parent = predecessor # a Link instance, not a name
            successor.supportingJoint = joint

    def readInertialData(self, linkNode):
        params = dict()
        paramsNode = linkNode.find('inertial')

        # Default inertia parameters if the URDF does not have the data
        if paramsNode == None :
            params['mass'] = 0.0
            params['xyz']  = (0.0, 0.0, 0.0)
            for m in URDFWrapper.iMomentsLabels :
                params[m] = 0.0
            return params

        mass = float(paramsNode.find('mass').get('value'))

        xyz = (0.0, 0.0, 0.0)
        originNode = paramsNode.find('origin')
        if originNode != None :
            comstr = originNode.get('xyz')
            if(comstr != None) :
                xyz = tuple([float(x) for x in comstr.split()])

            # We cannot deal with non-zero values for the 'rpy' attribute
            rpystr = originNode.get('rpy')
            if(rpystr != None) :
                tmp = [float(x) for x in rpystr.split()]
                if(sum(tmp) != 0) :
                    logger.warning('The rpy attribute in the inertial section is not yet supported (link ' + linkNode.get('name') + '). Ignoring it.')

        moments = paramsNode.find('inertia')
        for m in URDFWrapper.iMomentsLabels :
            params[m] = float(moments.get(m))

        params['mass'] = mass
        params['xyz']  = xyz
        return params


    def readJointFrameData(self, jointNode):
        params = dict()

        # URDF defaults:
        params['xyz'] = (0,0,0)
        params['rpy'] = (0,0,0)

        frameNode = jointNode.find('origin')
        if frameNode != None :
            xyz_node = frameNode.get('xyz')
            if xyz_node != None :
                params['xyz'] = tuple([float(x) for x in xyz_node.split()])
            rpy_node = frameNode.get('rpy')
            if rpy_node != None :
                params['rpy'] = tuple([float(x) for x in rpy_node.split()])

        axis_node = jointNode.find('axis')
        if axis_node != None :
            params['axis'] = tuple([float(x) for x in axis_node.get('xyz').split()])
        else :
            params['axis'] = (1,0,0) # URDF default

        return params


def linkOrigin(urdf, eelinkname):
    logger.debug("Entering urdfdbg_linkOrigin() function ...")
    H = np.identity(4)

    currentLink  = urdf.links[eelinkname]
    while currentLink is not None :
        currentJoint = currentLink.supportingJoint
        logger.debug("Link : " + currentLink.name)
        if currentJoint is not None :
            logger.debug("Joint: " + currentJoint.name)
            H = np.matmul( currentJoint.predec_H_joint , H )
        currentLink  = currentLink.parent

    print( np.round(H[:3,3], 5) )
