import sys
from ros_integration import main
if __name__=='__main__':
    sys.argv+=['--case','cancel_deploy','--legacy']
    raise SystemExit(main())
