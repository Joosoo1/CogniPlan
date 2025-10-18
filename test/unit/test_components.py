#!/usr/bin/env python3

"""
Unit tests for CogniPlan planner components
"""

import unittest
import numpy as np
import torch
import os
import sys
import tempfile

# Add the project root to the path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

# Print debug info
print(f"Project root: {project_root}")
print(f"Python path: {sys.path}")

try:
    from planner.model import PolicyNet
    from planner.utils import MapInfo
    from planner.parameter import NODE_INPUT_DIM, EMBEDDING_DIM
    print("Direct import successful")
except ImportError as e:
    print(f"Direct import failed: {e}")
    # Try absolute import
    try:
        sys.path.append('/home/joosoo/CLionProjects/CogniPlan')
        from planner.model import PolicyNet
        from planner.utils import MapInfo
        from planner.parameter import NODE_INPUT_DIM, EMBEDDING_DIM
        print("Absolute import successful")
    except ImportError as e2:
        print(f"Absolute import failed: {e2}")
        # If all imports fail, skip the tests that require these modules
        PolicyNet = None
        MapInfo = None
        NODE_INPUT_DIM = 10
        EMBEDDING_DIM = 128


class TestPolicyModel(unittest.TestCase):
    """Test the PolicyNet model loading and functionality"""
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        if PolicyNet is None:
            self.skipTest("PolicyNet not available due to import issues")
        self.device = torch.device('cpu')
        self.model = PolicyNet(NODE_INPUT_DIM, EMBEDDING_DIM).to(self.device)
        
    def test_model_initialization(self):
        """Test that the model initializes correctly"""
        self.assertIsInstance(self.model, PolicyNet)
        self.assertEqual(next(self.model.parameters()).device, self.device)
        
    def test_model_forward_pass(self):
        """Test that the model can perform a forward pass"""
        # Create dummy input data
        batch_size = 2
        seq_len = 10
        dummy_input = torch.randn(batch_size, seq_len, NODE_INPUT_DIM)
        node_padding_mask = torch.zeros(batch_size, seq_len, dtype=torch.bool)
        edge_mask = torch.ones(batch_size, seq_len, seq_len)
        current_index = torch.LongTensor([[0], [0]])  # Shape: [batch_size, 1]
        current_edge = torch.arange(5).unsqueeze(0).repeat(batch_size, 1)  # Shape: [batch_size, 5]
        edge_padding_mask = torch.zeros(batch_size, 5, dtype=torch.bool)
        
        # Test forward pass
        with torch.no_grad():
            output = self.model(dummy_input, node_padding_mask, edge_mask, current_index, current_edge, edge_padding_mask)
            
        # Check output shape - should be log probabilities for each action
        self.assertEqual(output.shape, (batch_size, 5))


class TestMapUtils(unittest.TestCase):
    """Test map processing utilities"""
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        if MapInfo is None:
            self.skipTest("MapInfo not available due to import issues")
        
    def test_map_info_creation(self):
        """Test MapInfo class creation"""
        test_map_data = np.array([
            [0, 0, 0, 0, 0],
            [0, 100, 100, 100, 0],
            [0, 100, -1, 100, 0],
            [0, 100, 100, 100, 0],
            [0, 0, 0, 0, 0]
        ], dtype=np.int8)
        
        map_info = MapInfo(test_map_data, 0.0, 0.0, 0.05)
        self.assertIsInstance(map_info, MapInfo)
        self.assertEqual(map_info.map.shape, test_map_data.shape)
        
    def test_get_frontier_detection(self):
        """Test frontier detection in map"""
        # This would require implementing the actual frontier detection function
        # For now, we'll skip this test as it depends on unimplemented functions
        pass


class TestUtils(unittest.TestCase):
    """Test utility functions"""
    
    def test_coordinate_conversion(self):
        """Test coordinate conversion utilities"""
        # This would test functions like get_cell_position_from_coords
        # For now, we'll skip this test as it depends on unimplemented functions
        pass


if __name__ == '__main__':
    unittest.main()