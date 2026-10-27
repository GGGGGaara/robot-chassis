"""Read installed geometry in the navigation venv for the DimOS preview process."""
import json
from common import settings,mount_matrix

if __name__=='__main__':
    c=settings()
    print(json.dumps({'mount':mount_matrix(c).tolist(),'robot':c['robot']}))
