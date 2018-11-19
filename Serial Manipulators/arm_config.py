def compute_arm_config(L1, L2, q0, q1):
  joint1_x = L1 * np.cos(q0)
  joint1_y = L2 * np.sin(q0)
  p2_x = joint1_x + L2 * np.cos(q0 + q1)
  p2_y = joint1_y + L2 * np.cos(q0 + q1)
  return joint1_x, joint1_y, p2_x, p2_y
  

#############################################################################
Create a jupyter notebook for this


#############################################################################
# Generate random link lengths and joint angles
# Note: because these are randomly generated on each run
# Every time you run the code you'll get a different result!
link1_length = np.random.random() * 30 + 20
link2_length = np.random.random() * 30 + 20
joint0_angle = np.random.random() * 2 * np.pi
joint1_angle = np.random.random() * 2 * np.pi

joint1_x, joint1_y, p2_x, p2_y = compute_arm_config(link1_length, link2_length, joint0_angle, joint1_angle)

print("joint0_angle =", round(joint0_angle * 180 / np.pi, 1), "degrees") 
print("joint1_angle =", round(joint1_angle * 180 / np.pi, 1),"degrees") 
print("End Effector at x =", round(p2_x, 1),"y =", round(p2_y, 1))
base_x = 0
base_y = 0
# Plot the links
plt.plot([base_x, joint1_x, p2_x], [base_y, joint1_y, p2_y])
# Plot the base as a blue square
plt.plot(base_x, base_y, 'bs', markersize=15, label='Base')
# Plot Joint-1 as a red circle
plt.plot(joint1_x, joint1_y, 'ro', markersize=15, label='Joint-1')
# Plot End Effector as a green triangle
plt.plot(p2_x, p2_y, 'g^', markersize=15, label='End Effector')
plt.xlim(-100, 100)
plt.ylim(-100, 100)
plt.legend(fontsize=15)
#plt.show() Uncomment to run locally
  
