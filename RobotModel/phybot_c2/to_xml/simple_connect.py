import mujoco

# 加载模型
model_path = '/home/jc/phybot_c1_v2/phybot_c2/xml/phybot_c2.xml'
model = mujoco.MjModel.from_xml_path(model_path)

# 打印模型body
print('Model Geometries:')
# for i in range(model.nbody):
#     print('************')
#     body = model.body(i)  # 获取每个几何体
#     print(f"Body {i}:")
#     print(f"  Name: {body.name}")
#     print(f"  geomnum: {body.geomnum}")
#     # print(f"  geomadr: {body.geom}")

# 打印模型的几何属性
print('Model Geometries:')
for i in range(model.ngeom):
    geom = model.geom(i)  # 获取每个几何体
    print(f"Geometry {i}:")
    print(f"  Name: {geom.name}")
    print(f"  Type: {geom.type}")

    print(f"  Pos: {geom.pos}")
    print(f"  Quaration: {geom.quat}")
    
    
    print(f"  Size: {geom.size}")

    # print(f"  geom: {geom}")
    # print(f"  Size: {geom.size}")
    # print(f"   {geom}")
    # print(f"  Type: {geom.type}")
    # print(f"  Connect_Type: {geom.contype}")
    # print(f"  Size: {geom.size}")
    # print(f"  Pos: {geom.pos}")
    # print(f"  Quaration: {geom.quat}")
    # print(f"  Friction: {geom.friction}")
    # print(f"  Group: {geom.group}")
    # print(f"  Contype: {geom.contype}")
    # print(f"  Conaffinity: {geom.conaffinity}")
    print('-' * 40)



