from django.http import HttpResponse
from ansible.runner import Runner
import json

def home(request):
    runner = Runner(module_name='ping',remote_user='im')
    result = runner.run()
    return HttpResponse(json.dumps(result))
