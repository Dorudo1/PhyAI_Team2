"""Workspace HTTP data comes exclusively from discovery snapshots."""
import json
import urllib.request
from junh_master.viewer.web import WorkspaceWeb


def test_local_workspace_server_updates_and_clears_session():
    web=WorkspaceWeb(0)
    try:
        assert web.server.server_address[0]=='127.0.0.1'
        root=f'http://127.0.0.1:{web.server.server_address[1]}'
        page=urllib.request.urlopen(root).read().decode()
        assert 'Discovered workspace' in page
        assert '/api/workspace' in page
        packet=json.load(urllib.request.urlopen(root+'/api/workspace'))
        assert packet['snapshot'] is None
        web.update({'workspace':{'session_id':'old','tracks':[{'id':'measured'}]}})
        packet=json.load(urllib.request.urlopen(root+'/api/workspace'))
        assert packet['snapshot']['workspace']['tracks'][0]['id']=='measured'
        assert packet['publisher_age']>=0
        web.update({'workspace':{'session_id':'new','tracks':[]}})
        packet=json.load(urllib.request.urlopen(root+'/api/workspace'))
        assert packet['snapshot']['workspace']['session_id']=='new'
        assert packet['snapshot']['workspace']['tracks']==[]
    finally:web.close()
