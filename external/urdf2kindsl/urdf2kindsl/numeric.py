import math
import numpy as np

close_enough_to_zero_R = 1e-10 # for the elements of rotation matrices

'''
Extrinsic rotations are about the axes of the original coordinate system, which
is assumed to remain motionless. This is the convention of the 'rpy' attribute
of the URDF format.
Intrinsic rotations are about the axes of a rotating coordinate system, attached
to the moving body, which changes its orientation after each individual
rotation. This is the convention of that 'rotation' attribute of the RobCoGen
format.
'''


def getR_intrinsicXYZ(rx, ry, rz):
    '''
    This is the rotation matrix **base_R_rotated**, where 'rotated' is obtained
    from 'base' with the **intrinsic** rotations rx, ry, and rz

                 cos(ry) cos(rz)                             - cos(ry) sin(rz)                     sin(ry)
    cos(rx) sin(rz) + sin(rx) sin(ry) cos(rz)    cos(rx) cos(rz) - sin(rx) sin(ry) sin(rz)    - sin(rx) cos(ry)
    sin(rx) sin(rz) - cos(rx) sin(ry) cos(rz)    cos(rx) sin(ry) sin(rz) + sin(rx) cos(rz)     cos(rx) cos(ry)
    '''
    sx = math.sin(rx)
    cx = math.cos(rx)
    sy = math.sin(ry)
    cy = math.cos(ry)
    sz = math.sin(rz)
    cz = math.cos(rz)
    return np.array(
        [ [cy*cz            , - cy*sz          ,sy      ],
          [cx*sz + cz*sx*sy , cx*cz - sx*sy*sz , - cy*sx],
          [sx*sz - cx*cz*sy , cx*sy*sz + cz*sx ,  cx*cy ] ] )

'''
Extract the intrinsic Euler angles XYZ from the rotation matrix **base_R_rotated**
'''
def getIntrinsicXYZFromR( Rin ) :
    # We need truncation, otherwise small coefficients will make the ratios
    # inside atan big enough to induce wrong results
    R = np.copy( Rin )
    R[ abs(R)<close_enough_to_zero_R ] = 0.0

    if R[0,2] != 1.0 and R[0,2] != -1.0 : # if not singular case, ie if not cos(ry) = 0
        ry = math.asin( R[0,2] )
        rx = math.atan2(-R[1,2], R[2,2])
        rz = math.atan2(-R[0,1], R[0,0])
    else :
        # In the singular case, we use other elements of the matrix to
        # reconstruct the angles. The expressions in these elements have the
        # form of sine/cosine of the sum of rx and rz; rz can however be set
        # arbitrarily to 0
        if R[0,2] == 1.0 :
            '''
            cos(ry) cos(rz)                      - cos(ry) sin(rz)                   1
    cos(rx) sin(rz) + sin(rx) cos(rz)    cos(rx) cos(rz) - sin(rx) sin(rz)           0
    sin(rx) sin(rz) - cos(rx) cos(rz)    cos(rx) sin(rz) + sin(rx) cos(rz)           0
            '''
            ry = math.pi/2
            rz = 0.0
            rx = math.atan2(R[1,0], -R[2,0]) # this is really rx+rz, but we set rz=0
        else : # R[0,2] = -1
            '''
            cos(ry) cos(rz)                      - cos(ry) sin(rz)                  -1
    cos(rx) sin(rz) - sin(rx) cos(rz)    cos(rx) cos(rz) + sin(rx) sin(rz)           0
    sin(rx) sin(rz) + cos(rx) cos(rz)   -cos(rx) sin(rz) + sin(rx) cos(rz)           0
            '''
            # R[1,1] = cos(rx-rz)
            # R[2,1] = sin(rx-rz)
            ry = - math.pi/2
            rz = 0.0
            rx = math.atan2( R[2,1], R[1,1])

    return (rx, ry, rz)


def getR_extrinsicXYZ(rx, ry, rz):
    '''
    This is the rotation matrix **base_R_rotated**, where 'rotated' is obtained from
    'base' with the **extrinsic** rotations rx, ry, and rz

    cos(ry) cos(rz)    sin(rx) sin(ry) cos(rz) - cos(rx) sin(rz)    sin(rx) sin(rz) + cos(rx) sin(ry) cos(rz)
    cos(ry) sin(rz)    sin(rx) sin(ry) sin(rz) + cos(rx) cos(rz)    cos(rx) sin(ry) sin(rz) - sin(rx) cos(rz)
       - sin(ry)                    sin(rx) cos(ry)                              cos(rx) cos(ry)
    '''
    sx = math.sin(rx)
    cx = math.cos(rx)
    sy = math.sin(ry)
    cy = math.cos(ry)
    sz = math.sin(rz)
    cz = math.cos(rz)
    return np.array(
        [[cy*cz,  cz*sx*sy - cx*sz,  sx*sz + cx*cz*sy],
         [cy*sz,  sx*sy*sz + cx*cz,  cx*sy*sz - cz*sx],
         [ - sy,        cy*sx     ,        cx*cy      ]] )


def _extrinsic2intrinsic_XYZ(erx, ery, erz):
    sx = math.sin(erx)
    cx = math.cos(erx)
    sy = math.sin(ery)
    cy = math.cos(ery)
    sz = math.sin(erz)
    cz = math.cos(erz)

    irx = math.atan2( sx*cz-cx*sy*sz, cx*cy)
    iry = math.asin ( sx*sz + cx*sy*cz )
    irz = math.atan2( cx*sz - sx*sy*cz, cy*cz )

    return (irx, iry, irz)

def _intrinsic2extrinsic_XYZ(irx, iry, irz):
    sx = math.sin(irx)
    cx = math.cos(irx)
    sy = math.sin(iry)
    cy = math.cos(iry)
    sz = math.sin(irz)
    cz = math.cos(irz)

    erx = math.atan2(cx*sy*sz + sx*cz, cx*cy)
    ery = math.asin(cx*sy*cz - sx*sz)
    erz = math.atan2(cx*sz+sx*sy*cz, cy*cz)

    return (erx, ery, erz)

def __cross_mx(r) :
    return np.array(
        [[ 0   , -r[2],  r[1] ],
         [ r[2],   0  , -r[0] ],
         [-r[1],  r[0],    0  ]] )

def rotoTranslateInertia(inertia, tr, R) :
    mass = inertia['mass']
    com  = inertia['com']
    vec  = com - tr

    com_x = __cross_mx(com)
    vec_x = __cross_mx(vec)

    ixx = inertia['Ix']
    iyy = inertia['Iy']
    izz = inertia['Iz']
    ixy = inertia['Ixy']
    ixz = inertia['Ixz']
    iyz = inertia['Iyz']
    tensor = np.array( [[ ixx, -ixy, -ixz],
                        [-ixy,  iyy, -iyz],
                        [-ixy, -iyz,  izz] ])
    tensor = tensor - mass * (np.matmul(com_x, com_x.T) - np.matmul(vec_x, vec_x.T))

    tensor2 = np.matmul(np.matmul(R, tensor), R.T)
    com2 = np.matmul(R, vec)
    ret = {}
    ret['mass'] = mass
    ret['com'] = com2
    ret['Ix']  =  tensor2[0,0]
    ret['Iy']  =  tensor2[1,1]
    ret['Iz']  =  tensor2[2,2]
    ret['Ixy'] = -tensor2[0,1]
    ret['Ixz'] = -tensor2[0,2]
    ret['Iyz'] = -tensor2[1,2]
    return ret