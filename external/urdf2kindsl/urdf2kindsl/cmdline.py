import sys, logging, argparse
from urdf2kindsl import urdf, convert, kindsl

logLevels = {}
logLevels['debug']   = logging.DEBUG
logLevels['info']    = logging.INFO
logLevels['warning'] = logging.WARNING
logLevels['error']   = logging.ERROR


def main() :
    argparser = argparse.ArgumentParser(
        description='Convert a URDF model to a Kinematics-DSL model')

    argparser.add_argument('urdf', metavar='URDF-input',
            help='path of the URDF input file')
    argparser.add_argument('-o', '--output',
            help='destination file (defaults to stdout)')
    argparser.add_argument('--digits',
            type=int,
            help='max number of digits for the fractional part of a real number (default 6)',
            default=6)
    argparser.add_argument('--pi-digits',
            type=int,
            help='number of digits of the fractional part of an angle used to determine if it is equal to PI (default 5)',
            default=5)
    argparser.add_argument('--floating', dest='floating', action='store_true', help='declare the base as floating')
    argparser.add_argument('--prune-fixed-joints', dest='prunefixed',
            action='store_true',
            help='prune fixed joints and child links - see also the following options')

    argparser.add_argument('--to-frames'   , dest='toframes', action='store_true')
    argparser.add_argument('--no-to-frames', dest='toframes', action='store_false',
            help='convert pruned links to custom frames in the parent link; defaults to true')

    argparser.add_argument('--lump-inertia', dest='lumpinertia', action='store_true')
    argparser.add_argument('--no-lump-inertia', dest='lumpinertia',
            action='store_false',
            help='''propagate up the tree the inertia of pruned links; defaults to true''')
    argparser.set_defaults(prunefixed=False)
    argparser.set_defaults(lumpinertia=True)
    argparser.set_defaults(toframes=True)
    argparser.set_defaults(floating=False)

    argparser.add_argument('--log-level', type=str, dest='loglevel',
            default='warning',
            help='logging level, chosen among debug, info, warning, error (defaults to warning)')
    group = argparser.add_argument_group('URDF inspection', 'Misc information about the given URDF (no conversion performed)')
    group.add_argument('--link-origin', metavar="LINK",
            type=str,
            help='print the origin of the frame of LINK in base coordinates, for the zero configuration'
            )
    args = argparser.parse_args()

    logging.basicConfig(level= logLevels[args.loglevel])

    ofile = sys.stdout
    if( args.output is not None) :
        ofile = open(args.output, 'w')

    urdfin = urdf.URDFWrapper( args.urdf )
    if args.link_origin is not None :
        urdf.linkOrigin(urdfin, args.link_origin)
    else :
        converterOpts = {
            convert.opt_key_prune : args.prunefixed,
            convert.opt_key_toframes : args.toframes,
            convert.opt_key_lumpi : args.lumpinertia
        }
        conv = convert.Converter( urdfin, converterOpts )
        form = kindsl.NumFormatter( round_digits=args.digits, pi_round_digits=args.pi_digits)
        ser  = kindsl.Serializer(ofile, numFormatter=form, floating=args.floating)
        ser.writeModel(conv)

if __name__ == '__main__':
    main()

