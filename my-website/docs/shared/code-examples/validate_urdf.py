#!/usr/bin/env python3

"""
URDF Validation Script
This script provides comprehensive validation for URDF files.
"""

import xml.etree.ElementTree as ET
import subprocess
import sys
import os
from math import isclose


def validate_urdf_comprehensive(urdf_file):
    """
    Perform comprehensive validation of a URDF file.
    """
    print(f"Validating {urdf_file}...")

    # 1. Check if file exists
    if not os.path.exists(urdf_file):
        print(f"ERROR: File {urdf_file} does not exist")
        return False

    # 2. Check syntax with check_urdf (if available)
    try:
        result = subprocess.run(['check_urdf', urdf_file],
                              capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            print(f"Syntax validation failed:\n{result.stderr}")
            return False
        else:
            print("✓ Basic syntax validation passed")
    except FileNotFoundError:
        print("⚠ check_urdf not found, skipping external syntax validation")
    except subprocess.TimeoutExpired:
        print("⚠ check_urdf timed out, continuing with internal validation")

    # 3. Parse XML internally
    try:
        tree = ET.parse(urdf_file)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"XML parsing failed: {e}")
        return False
    except Exception as e:
        print(f"Error reading URDF file: {e}")
        return False

    # 4. Validate URDF structure
    validation_results = {
        'basic_structure': True,
        'links_valid': True,
        'joints_valid': True,
        'inertial_valid': True,
        'kinematic_chain_valid': True
    }

    # Check robot name attribute
    robot_name = root.get('name')
    if not robot_name:
        print("⚠ Robot name not specified")
    else:
        print(f"✓ Robot name: {robot_name}")

    # Get all links and joints
    links = root.findall('.//link')
    joints = root.findall('.//joint')

    print(f"Found {len(links)} links and {len(joints)} joints")

    # Validate links
    link_names = []
    for link in links:
        link_name = link.get('name')
        if not link_name:
            print("ERROR: Link without name found")
            validation_results['links_valid'] = False
            continue

        if link_name in link_names:
            print(f"ERROR: Duplicate link name found: {link_name}")
            validation_results['links_valid'] = False
        else:
            link_names.append(link_name)

        # Validate inertial properties
        inertial = link.find('inertial')
        if inertial is None:
            print(f"⚠ Link '{link_name}' has no inertial properties")
        else:
            if not validate_inertial(inertial, link_name):
                validation_results['inertial_valid'] = False

    # Validate joints
    joint_names = []
    parent_child_pairs = []
    for joint in joints:
        joint_name = joint.get('name')
        joint_type = joint.get('type')

        if not joint_name:
            print("ERROR: Joint without name found")
            validation_results['joints_valid'] = False
            continue

        if joint_name in joint_names:
            print(f"ERROR: Duplicate joint name found: {joint_name}")
            validation_results['joints_valid'] = False
        else:
            joint_names.append(joint_name)

        if joint_type not in ['revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar']:
            print(f"ERROR: Invalid joint type '{joint_type}' in joint '{joint_name}'")
            validation_results['joints_valid'] = False

        # Check parent and child links exist
        parent_elem = joint.find('parent')
        child_elem = joint.find('child')

        if parent_elem is None or child_elem is None:
            print(f"ERROR: Joint '{joint_name}' missing parent or child specification")
            validation_results['joints_valid'] = False
            continue

        parent_link = parent_elem.get('link')
        child_link = child_elem.get('link')

        if parent_link not in link_names:
            print(f"ERROR: Joint '{joint_name}' references non-existent parent link '{parent_link}'")
            validation_results['joints_valid'] = False

        if child_link not in link_names:
            print(f"ERROR: Joint '{joint_name}' references non-existent child link '{child_link}'")
            validation_results['joints_valid'] = False

        if parent_link == child_link:
            print(f"ERROR: Joint '{joint_name}' has same parent and child link '{parent_link}'")
            validation_results['joints_valid'] = False

        # Store parent-child relationship
        parent_child_pairs.append((parent_link, child_link))

    # Validate kinematic chain (no loops, single root)
    if not validate_kinematic_chain(parent_child_pairs, link_names):
        validation_results['kinematic_chain_valid'] = False

    # Summary
    all_valid = all(validation_results.values())

    print("\nValidation Summary:")
    print(f"  Basic Structure: {'✓' if validation_results['basic_structure'] else '✗'}")
    print(f"  Links: {'✓' if validation_results['links_valid'] else '✗'}")
    print(f"  Joints: {'✓' if validation_results['joints_valid'] else '✗'}")
    print(f"  Inertial: {'✓' if validation_results['inertial_valid'] else '✗'}")
    print(f"  Kinematic Chain: {'✓' if validation_results['kinematic_chain_valid'] else '✗'}")

    if all_valid:
        print("\n✓ URDF validation passed!")
        return True
    else:
        print("\n✗ URDF validation failed!")
        return False


def validate_inertial(inertial_elem, link_name):
    """
    Validate inertial properties of a link.
    """
    mass_elem = inertial_elem.find('mass')
    inertia_elem = inertial_elem.find('inertia')

    if mass_elem is None:
        print(f"⚠ Link '{link_name}' has inertial element but no mass")
        return False

    try:
        mass_value = float(mass_elem.get('value'))
        if mass_value <= 0:
            print(f"⚠ Link '{link_name}' has non-positive mass: {mass_value}")
            return False
    except (ValueError, TypeError):
        print(f"⚠ Link '{link_name}' has invalid mass value")
        return False

    if inertia_elem is None:
        print(f"⚠ Link '{link_name}' has inertial element but no inertia matrix")
        return False

    try:
        ixx = float(inertia_elem.get('ixx', 0))
        ixy = float(inertia_elem.get('ixy', 0))
        ixz = float(inertia_elem.get('ixz', 0))
        iyy = float(inertia_elem.get('iyy', 0))
        iyz = float(inertia_elem.get('iyz', 0))
        izz = float(inertia_elem.get('izz', 0))
    except (ValueError, TypeError):
        print(f"⚠ Link '{link_name}' has invalid inertia values")
        return False

    # Check if inertia matrix is physically valid (positive definite)
    # For a valid inertia matrix, diagonal elements should be positive
    # and satisfy triangle inequalities: ixx + iyy >= izz, etc.
    if ixx <= 0 or iyy <= 0 or izz <= 0:
        print(f"⚠ Link '{link_name}' has non-positive diagonal inertia: ixx={ixx}, iyy={iyy}, izz={izz}")
        return False

    # Check triangle inequalities
    if not (ixx + iyy >= izz and iyy + izz >= ixx and izz + ixx >= iyy):
        print(f"⚠ Link '{link_name}' has invalid inertia triangle inequality")
        return False

    # Check if off-diagonal elements are reasonable (should be small relative to diagonal)
    tolerance = 0.9  # Allow off-diagonal elements up to 90% of the corresponding diagonal
    if abs(ixy) > tolerance * min(ixx, iyy) or \
       abs(ixz) > tolerance * min(ixx, izz) or \
       abs(iyz) > tolerance * min(iyy, izz):
        print(f"⚠ Link '{link_name}' has suspiciously large off-diagonal inertia elements")
        # This is a warning, not an error, as it might be valid in some cases

    return True


def validate_kinematic_chain(parent_child_pairs, all_link_names):
    """
    Validate that the kinematic chain forms a valid tree structure.
    """
    if not parent_child_pairs and len(all_link_names) == 0:
        print("⚠ No links or joints found")
        return False

    if not parent_child_pairs and len(all_link_names) > 0:
        # Only one link with no joints is valid
        if len(all_link_names) == 1:
            print("✓ Single link with no joints is valid")
            return True
        else:
            print("ERROR: Multiple links with no joints")
            return False

    # Build parent map
    child_to_parent = {}
    for parent, child in parent_child_pairs:
        if child in child_to_parent:
            print(f"ERROR: Link '{child}' has multiple parents")
            return False
        child_to_parent[child] = parent

    # Find root links (those that are not children of any joint)
    all_children = set(child for parent, child in parent_child_pairs)
    all_parents = set(parent for parent, child in parent_child_pairs)
    root_candidates = set(all_link_names) - all_children

    if len(root_candidates) == 0:
        print("ERROR: No root link found - possible loop in kinematic chain")
        return False

    if len(root_candidates) > 1:
        print(f"⚠ Multiple root links found: {root_candidates}")
        # This might be valid in some cases, but let's warn
        # For a single robot, there should typically be one root

    # Check for cycles by traversing from each link to root
    for link in all_link_names:
        visited = set()
        current = link
        while current in child_to_parent:
            if current in visited:
                print(f"ERROR: Cycle detected in kinematic chain involving link '{current}'")
                return False
            visited.add(current)
            current = child_to_parent[current]

    print("✓ Kinematic chain validation passed")
    return True


def generate_urdf_report(urdf_file):
    """
    Generate a detailed report about the URDF file.
    """
    print(f"\n--- URDF Report for {urdf_file} ---")

    try:
        tree = ET.parse(urdf_file)
        root = tree.getroot()
    except Exception as e:
        print(f"Could not parse URDF file: {e}")
        return

    robot_name = root.get('name', 'unnamed')
    print(f"Robot Name: {robot_name}")

    # Count elements
    links = root.findall('.//link')
    joints = root.findall('.//joint')
    materials = root.findall('.//material')
    gazebo_elements = root.findall('.//gazebo')

    print(f"Total Links: {len(links)}")
    print(f"Total Joints: {len(joints)}")
    print(f"Materials: {len(materials)}")
    print(f"Gazebo Elements: {len(gazebo_elements)}")

    # Analyze joint types
    joint_types = {}
    for joint in joints:
        jtype = joint.get('type', 'unknown')
        joint_types[jtype] = joint_types.get(jtype, 0) + 1

    print("\nJoint Type Distribution:")
    for jtype, count in joint_types.items():
        print(f"  {jtype}: {count}")

    # Analyze link properties
    links_with_visual = 0
    links_with_collision = 0
    links_with_inertial = 0

    for link in links:
        if link.find('visual') is not None:
            links_with_visual += 1
        if link.find('collision') is not None:
            links_with_collision += 1
        if link.find('inertial') is not None:
            links_with_inertial += 1

    print(f"\nLink Property Distribution:")
    print(f"  With Visual: {links_with_visual}")
    print(f"  With Collision: {links_with_collision}")
    print(f"  With Inertial: {links_with_inertial}")

    # Check for common issues
    print(f"\nPotential Issues:")
    if links_with_inertial < len(links):
        missing_inertial = len(links) - links_with_inertial
        print(f"  - {missing_inertial} link(s) missing inertial properties")

    if links_with_visual == 0:
        print("  - No visual elements found")

    if links_with_collision == 0:
        print("  - No collision elements found")


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 validate_urdf.py <urdf_file>")
        print("Example: python3 validate_urdf.py simple_humanoid.urdf")
        sys.exit(1)

    urdf_file = sys.argv[1]

    # Perform validation
    is_valid = validate_urdf_comprehensive(urdf_file)

    # Generate detailed report
    generate_urdf_report(urdf_file)

    # Exit with appropriate code
    sys.exit(0 if is_valid else 1)


if __name__ == "__main__":
    main()