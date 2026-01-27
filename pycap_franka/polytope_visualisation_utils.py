from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

def create_polytope_triangles_msg(vertices, faces, position, timestamp):
    """Publish polytope as a TRIANGLE_LIST marker"""
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = timestamp
    marker.ns = "polytope"
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    
    # Set position to end-effector position
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.pose.orientation.w = 1.0
    
    # Scale
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    
    # Color (semi-transparent blue)
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 0.3
    
    # Add vertices for each triangle face
    from geometry_msgs.msg import Point
    for face in faces:
        for vertex_idx in face:
            point = Point()
            point.x = vertices[0, vertex_idx]
            point.y = vertices[1, vertex_idx]
            point.z = vertices[2, vertex_idx]
            marker.points.append(point)
    
    return marker

def create_polytope_edges_msg(vertices, faces, position, timestamp):
    # Publish edges (LINE_STRIP)
    edge_marker = Marker()
    edge_marker.header.frame_id = "base"
    edge_marker.header.stamp = timestamp
    edge_marker.ns = "polytope_edges"
    edge_marker.id = 1
    edge_marker.type = Marker.LINE_LIST
    edge_marker.action = Marker.ADD
    
    # Set position to end-effector position
    edge_marker.pose.position.x = position[0]
    edge_marker.pose.position.y = position[1]
    edge_marker.pose.position.z = position[2]
    edge_marker.pose.orientation.w = 1.0
    
    # Scale (line width)
    edge_marker.scale.x = 0.005  # Line width
    
    # Color (solid black or dark blue)
    edge_marker.color.r = 0.0
    edge_marker.color.g = 0.0
    edge_marker.color.b = 0.0
    edge_marker.color.a = 0.7
    
    # Add edge vertices (each face has 3 edges for triangles)
    for face in faces:
        # For each triangular face, draw 3 edges
        for i in range(len(face)):
            # Start point
            point1 = Point()
            point1.x = vertices[0, face[i]]
            point1.y = vertices[1, face[i]]
            point1.z = vertices[2, face[i]]
            
            # End point (next vertex, wrapping around)
            point2 = Point()
            next_idx = (i + 1) % len(face)
            point2.x = vertices[0, face[next_idx]]
            point2.y = vertices[1, face[next_idx]]
            point2.z = vertices[2, face[next_idx]]
            
            edge_marker.points.append(point1)
            edge_marker.points.append(point2)
    
    return edge_marker

def orient_faces_outward(vertices, faces):
    """
    Orient face vertices so that normals point outward from the polytope center.
    vertices: 3xN array of vertex coordinates
    faces: list of face vertex indices
    Returns: list of reoriented faces
    """
    # Compute centroid of the polytope
    centroid = np.mean(vertices, axis=1)
    
    oriented_faces = []
    for face in faces:
        # Get the three vertices of the triangle
        v0 = vertices[:, face[0]]
        v1 = vertices[:, face[1]]
        v2 = vertices[:, face[2]]
        
        # Compute face center
        face_center = (v0 + v1 + v2) / 3.0
        
        # Compute face normal using cross product
        edge1 = v1 - v0
        edge2 = v2 - v0
        normal = np.cross(edge1, edge2)
        
        # Vector from centroid to face center
        centroid_to_face = face_center - centroid
        
        # Check if normal points outward (dot product > 0)
        if np.dot(normal, centroid_to_face) < 0:
            # Flip the face by swapping two vertices
            oriented_faces.append([face[0], face[2], face[1]])
        else:
            oriented_faces.append(face)
    
    return oriented_faces