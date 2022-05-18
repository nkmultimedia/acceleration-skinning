import maya.cmds as mc
import maya.mel as mel
import math
import sys

# Initial setup
print "Starting script..."
mesh_name = ''
dialogTitle = "Velocity Skinning"
root = False
try:
    mesh_name = velocity_skinning_target_mesh
except:
    # ask user to select a mesh to extract animation from
    msg = "Set target mesh before running this script.\n"
    mc.confirmDialog(title=dialogTitle, message=msg, messageAlign="center", button="OK")
    sys.exit("Target model undefined")
    
try:
    root = velocity_skinning_target_skeleton_root
except:
    # ask user to select a mesh to extract animation from
    msg = "Set targe skeleton root node before running this script.\n"
    mc.confirmDialog(title=dialogTitle, message=msg, messageAlign="center", button="OK")
    sys.exit("Target skeleton undefined")

mc.undoInfo(stateWithoutFlush=False)
progressWindow = mc.window(title="Exporting Animation")
mc.columnLayout()
progressControl = mc.progressBar(maxValue=100, width=500)
mc.showWindow( progressWindow )

def velocity_skinning_cleanup():
    mc.currentUnit(time=initial_time_unit)
    mc.deleteUI(progressWindow)
    mc.select(None)
    mc.undoInfo(stateWithoutFlush=True)
    
#########################################################################
######################## HELPER FUNCTIONS ###############################
#########################################################################

def get_joints_tree_dfs(root):
    '''
        Recursively traverse the node tree in a DFS manner
    '''
    def visit_children(node):

        children = mc.listRelatives(node)
        if children == None:
            return
        
        for child in children:
            if not child in joints_dfs:
                joints_dfs.append(child)
            visit_children(child)
    
    joints_dfs = [root]
    visit_children(root)
    return joints_dfs
    
def get_matrix(jointName):
    # arr of all elements
    matrixArr = mc.getAttr(jointName + ".matrix")
    
    # transpose matrix
    transposedMatrix = []
    for i in range(4):
        row = []
        for j in range(4):
            row.append(matrixArr[j*4 + i])
        transposedMatrix.append(row)
    return transposedMatrix
    
def matrix_to_translation(T):
    return [T[0][3],T[1][3],T[2][3]]
    

def matrix_to_quaternion(T):

    x,y,z,w = 0,0,0,0

    trace = T[0][0] + T[1][1] + T[2][2]; 
    if trace > 0 :
        s = 0.5 / math.sqrt(trace+ 1.0)
        w = 0.25 / s
        x = ( T[2][1] - T[1][2] ) * s
        y = ( T[0][2] - T[2][0] ) * s
        z = ( T[1][0] - T[0][1] ) * s
    else:
        if (T[0][0] > T[1][1]) and (T[0][0] > T[2][2]):
            s = 2.0 * math.sqrt( 1.0 + T[0][0] - T[1][1] - T[2][2])
            w = (T[2][1] - T[1][2] ) / s
            x = 0.25 * s
            y = (T[0][1] + T[1][0] ) / s
            z = (T[0][2] + T[2][0] ) / s
        elif T[1][1] > T[2][2] :
            s = 2.0 * math.sqrt( 1.0 + T[1][1] - T[0][0] - T[2][2])
            w = (T[0][2] - T[2][0] ) / s
            x = (T[0][1] + T[1][0] ) / s
            y = 0.25 * s
            z = (T[1][2] + T[2][1] ) / s
        else:
            s = 2.0 * math.sqrt( 1.0 + T[2][2] - T[0][0] - T[1][1] )
            w = (T[1][0] - T[0][1] ) / s
            x = (T[0][2] + T[2][0] ) / s
            y = (T[1][2] + T[2][1] ) / s
            z = 0.25 * s

    return (x,y,z,w)
    
def generate_symmetry():
    symmetry = []
    N_names = len(skeleton_name)
    for k1 in range(N_names):
        for k2 in range(k1+1,N_names):
            name1 = skeleton_name[k1]
            name2 = skeleton_name[k2]
    
            if name1 == name2.replace('_L_','_R_') or name1 == name2.replace('_R_','_L_'):
                symmetry.append( (k1,k2) )
            elif name1.endswith('_L') and name1==name2.replace('_R','_L'):
                symmetry.append( (k1,k2) )
            elif name1.endswith('_R') and name1==name2.replace('_L','_R'):
                symmetry.append( (k1,k2) )
                
    with open("skeleton_symmetry",'w') as fid:
        for s in symmetry:
            fid.write( str(s[0])+" "+str(s[1])+"\n" )
            
def load_bind_pose(mesh_name):
    mc.select(mesh_name)
    mel.eval('gotoBindPose')
    mc.select(None)
            
def get_keyframe_list(obj_name):
    all_key_times = mc.keyframe(obj_name, query=True, timeChange=True)
    if all_key_times is None:
        return [0, end_time/2, end_time]
        
    unique_key_times = [0]
    # get list of which frames have keys
    for key in all_key_times:
        if key not in unique_key_times:
            unique_key_times.append(key)
            
    if end_time not in unique_key_times:
        unique_key_times.append(end_time)
    return unique_key_times
    
def getSkinCluster():
    skin_cluster_node = ""
    mc.select(mesh_name)
    selected_items = mc.ls(selection=True)
    relative_shape_nodes = mc.listRelatives(selected_items[0], shapes=True)
    skinning_cluster_node = mc.findType(root, type="skinCluster", deep=True, forward=True)

    #verify result
    target_shape_node = mc.skinCluster(skinning_cluster_node, query=True, geometry=True)[0]
    for node in relative_shape_nodes:
        if node == target_shape_node: # confirmed: skinning cluster found
            return skinning_cluster_node[0]
            
    sys.exit("Could not find attached skinCluster node")
    
#########################################################################
######################## MAIN PROCEDURE #################################
#########################################################################


skeleton = mc.ls(type='joint')
node_to_visit = [(root, -1)]
skeleton_name = []
skeleton_parent_name = []
skeleton_translation = []
skeleton_rotation = []
skeleton_animation = []
keys = []
keys_per_joint = []
unique_key_times = []
initial_time_unit = mc.currentUnit(time=True, query=True)
mc.currentUnit(time='sec')
joints_dfs = get_joints_tree_dfs(root)
joint_count = len(joints_dfs)
vertex_count = cmds.polyEvaluate(mesh_name, vertex=True)
end_time = mc.playbackOptions(query=True, maxTime=True)

print "Initialization complete. Extracting animation data..."
# get list of joints and parent names (BFS)
while len(node_to_visit) > 0:
    load_bind_pose(mesh_name)
    
    # prepare needed data
    node,parent_name = node_to_visit.pop(0)
    name = mc.ls(node)[0]
    T = get_matrix(name)
    t = matrix_to_translation(T)
    q = matrix_to_quaternion(T)
    q_norm = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    assert abs(q_norm - 1) < 1e-2

    
    # store data in relevent structure
    skeleton_name.append(name)
    skeleton_parent_name.append(parent_name)    
    skeleton_translation.append(t)
    skeleton_rotation.append(q)
    
    # get animation of current joint
    joint_keys = []
    key_count = mc.keyframe( name, query=True, keyframeCount=True)
    if key_count < 3:
        key_count = 3
    keys_per_joint.append(key_count)
    unique_key_times = get_keyframe_list(name)
    
    for key in unique_key_times:
        mc.currentTime(key)
        T = get_matrix(name)
        t = matrix_to_translation(T)
        q = matrix_to_quaternion(T)
        q_norm = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        assert abs(q_norm - 1) < 1e-2
        
        joint_keys.append((key, t[0], t[1], t[2], q[0], q[1], q[2], q[3]))
    skeleton_animation.append(joint_keys)
    
    # update progress bar
    mc.progressBar(progressControl, edit=True, step=100/joint_count)
    
    # populate next level of nodes
    children = mc.listRelatives(node)
    if children == None:
        continue
        
    for child in children:
        node_to_visit.append( (child, name) )

# generate map of "joint_name": index
name_to_indice = {-1:-1}
counter = 0

for name in skeleton_name:
    name_to_indice[name]=counter
    counter = counter+1

# map parent names to indices
skeleton_parent_connectivity = [name_to_indice[parent_name] for parent_name in skeleton_parent_name]

# generate skinning weights
skinning_joint = []
skinning_weights = []
skinning_cluster_node = getSkinCluster()


print "Extracting skinning weights..."
for i in range(vertex_count):
    target_obj = mesh_name + ".vtx[" + str(i) + "]"
    vertex_weights = []
    
    for j in joints_dfs:
        try:
            weight = mc.skinPercent(skinning_cluster_node, target_obj, query=True, transform=j)
        except:
            # current joint is not an influence object for this skin; remove it from the joint list
            joints_dfs.remove(j)
            continue
        vertex_weights.append(weight)
    
    weights = []
    joints_idx = []
    
    for j in range(len(vertex_weights)):
        w = vertex_weights[j]
        if w == 0:
            continue
        
        weights.append(w)
        j_name = joints_dfs[j]
        joints_idx.append(name_to_indice[j_name])
    
    skinning_joint.append(joints_idx)
    skinning_weights.append(weights)
       
        
print "Writing data to files..."
#########################################################################
######################## WRITE TO FILE ##################################
#########################################################################
basePath = mc.workspace(fullName=True)
customDirName = "velocitySkinning"
os.chdir(basePath + "\data")
if not os.path.exists(customDirName):
    os.mkdir(customDirName)
os.chdir(customDirName)

#joint_count = len(skeleton_name)
#with open('skeleton_connectivity','w') as f:
#    for i in range(joint_count):
#        line = str(i) + ' ' + str(skeleton_parent_connectivity[i]) + ' ' + skeleton_name[i] + '\n'
#        f.write(line)

with open('skeleton_joint_name','w') as f:
    for name in skeleton_name:
        f.write(name+'\n')
        
with open('skeleton_connectivity_parent','w') as f:
    for parent in skeleton_parent_connectivity:
        f.write(str(parent)+' ')
        
#with open('skeleton_geometry_local','w') as f:
#    for i in range(joint_count):
#        t = skeleton_translation[i]
#        q = skeleton_rotation[i]
#        line = str(t[0]) + ' ' + str(t[1]) + ' ' + str(t[2]) + ' '
#        line += str(q[0]) + ' ' + str(q[1]) + ' ' + str(q[2]) + ' ' + str(q[3]) + '\n'
#        f.write(line)
        
with open('skeleton_rest_pose_rotation','w') as f:
    for r in skeleton_rotation:
        f.write(str(r[0])+' '+str(r[1])+' '+str(r[2])+' '+str(r[3])+'\n')
        
with open('skeleton_rest_pose_translation','w') as f:
    for t in skeleton_translation:
        f.write(str(t[0])+' '+str(t[1])+' '+str(t[2])+'\n')
        
with open('rig_weights','w') as f:
    for weights in skinning_weights:
        for w in weights:
            f.write(str(w)+' ')
        f.write('\n')
with open('rig_joint','w') as f:
    for joints in skinning_joint:
        for j in joints:
            f.write(str(j)+' ')
        f.write('\n')
        
with open('skeleton_animation','w') as f:
    for j in skeleton_animation:
        f.write(str(len(j)) + '\n')
        for k in j:
            f.write( str(k[0]) + ' ' + str(k[1]) + ' ' + str(k[2]) + ' ' + str(k[3]) + ' ' + str(k[4]) + ' ' + str(k[5]) + ' ' + str(k[6]) + ' ' + str(k[7]) + '\n' )
print "Execution complete. Reseting modifications..."

mc.progressBar(progressControl, edit=True, progress=100)
dialogTitle = "Velocity Skinning"        
#question = "Is this rig symmetrical?"
#is_rig_symmetrical = mc.confirmDialog(title=dialogTitle, message=question, messageAlign="center", button=["Yes", "No"], defaultButton="Yes", cancelButton="No", dismissString="No")
#if(is_rig_symmetrical == "Yes"):
#    generate_symmetry()
        
msg = "Rigging data extraction complete. All required files have been saved to: \n\n" + os.getcwd() + "\n"
mc.confirmDialog(title=dialogTitle, message=msg, messageAlign="center", button="OK")
velocity_skinning_cleanup()
print "Cleanup complete."