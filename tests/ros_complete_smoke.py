import sys
from ros_integration import main
if __name__=='__main__':
    sys.argv+=['--case','complete']
    raise SystemExit(main())
