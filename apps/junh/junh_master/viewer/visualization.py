"""Build compact viewer primitives from the same snapshot used by the master."""

COLORS = {'observed':(.2,.65,.3,.6), 'uncertain':(1.,.65,.1,.65),
          'unobserved':(.5,.5,.5,.2)}


def primitives(envelope, now=None, stale_seconds=10.):
    workspace = envelope['workspace']
    frame = workspace['frame']
    now = workspace['stamp'] if now is None else now
    plane = workspace['table']
    items = []
    if plane:
        normal, offset = plane['normal'], plane['offset']
        size = workspace['coverage']['cell_size']
        for cell in workspace['coverage']['cells']:
            x,y = [(i+.5)*size for i in cell['index']]
            z = -(normal[0]*x+normal[1]*y+offset)/normal[2]
            age = None if cell['last_observed'] is None else max(0.,now-cell['last_observed'])
            color = COLORS[cell['state']]
            if age is not None and age > stale_seconds and not envelope['search'].get('result'):
                color = (.6,.25,.6,.4)
            items.append({'kind':'cell','position':[x,y,z], 'scale':[size,size,.001],
                          'quaternion':[1.,0.,0.,0.], 'color':color, 'frame':frame})
    for track in workspace['tracks'] + workspace.get('hypotheses',[]):
        age = max(0.,now-track['last_observed'])
        color = (.2,.5,1.,.65) if track['id'] and track['resolved'] else (1.,.3,.1,.65)
        if age > stale_seconds and not envelope['search'].get('result'):
            color = (.6,.25,.6,.5)
        items.append({'kind':'block','position':track['position'],'scale':track['size'],
                      'quaternion':track['quaternion'],'color':color,'frame':frame})
        items.append({'kind':'label','position':[track['position'][0],track['position'][1],
                      track['position'][2]+track['size'][2]/2+.025],
                      'text':f'{(track["id"] or track.get("hypothesis_id"))} | {track["quality"]} | {age:.1f}s',
                      'scale':[0.,0.,.015], 'quaternion':[1.,0.,0.,0.],
                      'color':(1.,1.,1.,1.),'frame':frame})
    search = envelope['search']
    items.append({'kind':'label','position':[0.,0.,.15],
        'text':f'{search["state"]}: {search["reason"]}\n'
               f'coverage {workspace["coverage"]["observed_fraction"]:.0%}; '
               f'map {workspace["revision"]}; {envelope.get("input_mode","unknown")}\n'
               'Discovery geometry only; no motion authorization',
        'scale':[0.,0.,.02], 'quaternion':[1.,0.,0.,0.],
        'color':(1.,1.,1.,1.),'frame':frame})
    return items
