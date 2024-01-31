from graphviz import Digraph

g = Digraph('G', filename="teste", format="png")

# NOTE: the subgraph name needs to begin with 'cluster' (all lowercase)
#       so that Graphviz recognizes it as a special cluster subgraph

with g.subgraph(name='cluster_0') as c:
	c.attr(label='Robot <Nome>')
	c.attr(color='blue')
	c.node_attr['style'] = 'filled'
	# c.edges([('cb0', 'cb1')])

	with c.subgraph(name='cluster_1') as d:
		d.attr(label='Input')
		d.attr(color='blue')
		d.node_attr['style'] = 'filled'
		d.node('Encoders')
		d.node('Lidar')
		d.node('Camera')

	with c.subgraph(name='cluster_2') as d:
		d.attr(label='CPU Raspberry')
		d.attr(color='red')
		# c.attr(style='filled', color='lightgrey')
		# c.node_attr.update(style='filled', color='white')
		d.node('Control')
		d.node('SLAM')
		d.edges([('Encoders', 'Control'), ('Lidar', 'Control'), ('SLAM', 'Control'), ('Control', 'SLAM')])

	with c.subgraph(name='cluster_3') as d:
		d.attr(label='Output')
		d.attr(color='blue')
		# c.attr(style='filled', color='lightgrey')
		# c.node_attr.update(style='filled', color='white')
		d.node('Motors')
		d.edges([('Control', 'Motors')])


with g.subgraph(name='cluster_4') as c:
	c.attr(label='CPU Central')
	c.attr(color='red')
	c.node_attr['style'] = 'filled'
	c.node('Yolo')
	c.edges([('Camera', 'Yolo')])

g.render(filename='architecture')
# g.view()