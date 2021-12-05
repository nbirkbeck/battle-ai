import bpy

output_file = open('/tmp/world.textpb', 'w')
for oname, o in bpy.data.objects.items():
    if o.type == 'MESH':
        loc = [o.location[0], o.location[2], o.location[1]]
        scale = [o.scale[0] * 2, o.scale[2] * 2, o.scale[1] * 2]
        data_name = o.data.name
        if data_name.find('Armor') == 0:
            amount = float(data_name.split('.')[1])
            power_up_template = 'power_ups { pos { x:%f y:%f z:%f }  type: POWER_UP_ARMOR amount: %f}\n'
            output_file.write(power_up_template % (loc[0], loc[1], loc[2], amount))
        elif data_name.find('Health') == 0:
            amount = float(data_name.split('.')[1])
            power_up_template = 'power_ups { pos { x:%f y:%f z:%f }  type: POWER_UP_HEALTH amount: %f}\n'
            output_file.write(power_up_template % (loc[0], loc[1], loc[2], amount))
        elif data_name.find('Spawn') == 0:
            spawn_template = 'spawn_points { pos { x:%f y:%f z:%f } }\n'
            output_file.write(spawn_template % (loc[0], loc[1], loc[2]))
        elif len(o.data.vertices) == 8:
            box_template = 'boxes { pos { x:%f y:%f z:%f } size { x:%f y:%f z:%f } }\n'
            print(o.name)
            output_file.write(box_template % (loc[0], loc[1], loc[2], scale[0], scale[1], scale[2]))
        else:
            print('Unknown object: %s,%s\n' % (o.name, o.blendata.name))

output_file.close()