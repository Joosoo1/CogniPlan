#!/usr/bin/env python3
"""
Unit tests for the JointTrainer class
"""

import unittest
import torch
import numpy as np
import sys
import os
import tempfile

# Add the project root to the path
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

try:
    from planner.joint_trainer import JointTrainer, create_joint_trainer
    from planner.parameter import NODE_INPUT_DIM, EMBEDDING_DIM
    HAS_REQUIRED_MODULES = True
except ImportError as e:
    print(f"Import error: {e}")
    HAS_REQUIRED_MODULES = False


class TestJointTrainer(unittest.TestCase):
    """Test the JointTrainer class"""
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        if not HAS_REQUIRED_MODULES:
            self.skipTest("Required modules not available")
            
        # Create a minimal config for testing
        self.config = {
            'use_gpu': False,
            'use_cuda': False,
            'generator_config': {
                'input_dim': 1,
                'ngf': 16
            },
            'generator_lr': 1e-4,
            'policy_lr': 1e-4,
            'q_lr': 1e-4
        }
        
    def test_initialization(self):
        """Test that the JointTrainer initializes correctly"""
        trainer = JointTrainer(self.config)
        
        # Check that all networks are initialized
        self.assertIsNotNone(trainer.generator)
        self.assertIsNotNone(trainer.policy_net)
        self.assertIsNotNone(trainer.q_net)
        self.assertIsNotNone(trainer.target_q_net)
        
        # Check that optimizers are initialized
        self.assertIsNotNone(trainer.generator_optimizer)
        self.assertIsNotNone(trainer.policy_optimizer)
        self.assertIsNotNone(trainer.q_optimizer)
        
    def test_forward_pass(self):
        """Test that the forward pass works correctly"""
        trainer = JointTrainer(self.config)
        
        # Create dummy input data
        batch_size = 2
        height, width = 64, 64
        x = torch.randn(batch_size, 1, height, width)
        mask = torch.zeros(batch_size, 1, height, width)
        mask[:, :, 20:40, 20:40] = 1.0  # Create a masked region
        onehot = torch.randn(batch_size, 3, 1, 1)
        
        # Test forward pass
        with torch.no_grad():
            outputs = trainer.forward_pass(x, mask, onehot)
            
        # Check that all expected outputs are present
        self.assertIn('generated_map', outputs)
        self.assertIn('inpainted_result', outputs)
        self.assertIn('policy_output', outputs)
        self.assertIn('q_values', outputs)
        self.assertIn('graph_representation', outputs)
        
        # Check output shapes
        self.assertEqual(outputs['generated_map'].shape, (batch_size, 1, height, width))
        self.assertEqual(outputs['inpainted_result'].shape, (batch_size, 1, height, width))
        
    def test_compute_loss(self):
        """Test that the loss computation works correctly"""
        trainer = JointTrainer(self.config)
        
        # Create dummy predictions and targets
        batch_size = 2
        height, width = 64, 64
        predictions = {
            'inpainted_result': torch.randn(batch_size, 1, height, width)
        }
        targets = torch.randn(batch_size, 1, height, width)
        rewards = torch.randn(batch_size, 1)
        actions = torch.randint(0, 5, (batch_size, 1))
        
        # Test generator-only loss
        losses = trainer.compute_loss(predictions, targets, rewards, actions)
        self.assertIn('generator_loss', losses)
        self.assertIn('total_loss', losses)
        self.assertIsInstance(losses['generator_loss'].item(), float)
        self.assertIsInstance(losses['total_loss'].item(), float)
        
    def test_train_step(self):
        """Test that a training step works correctly"""
        trainer = JointTrainer(self.config)
        
        # Create dummy batch data
        batch_size = 2
        height, width = 64, 64
        x = torch.randn(batch_size, 1, height, width)
        mask = torch.zeros(batch_size, 1, height, width)
        mask[:, :, 20:40, 20:40] = 1.0
        onehot = torch.randn(batch_size, 3, 1, 1)
        targets = torch.randn(batch_size, 1, height, width)
        rewards = torch.randn(batch_size, 1)
        actions = torch.randint(0, 5, (batch_size, 1))
        
        batch_data = (x, mask, onehot, targets, rewards, actions)
        
        # Test training step
        metrics = trainer.train_step(batch_data)
        
        # Check that metrics are returned
        self.assertIn('loss', metrics)
        self.assertIn('generator_loss', metrics)
        self.assertIsInstance(metrics['loss'], float)
        self.assertIsInstance(metrics['generator_loss'], float)
        
    def test_save_load_checkpoint(self):
        """Test that saving and loading checkpoints works correctly"""
        trainer = JointTrainer(self.config)
        
        # Create a temporary file for the checkpoint
        with tempfile.NamedTemporaryFile(suffix='.pth', delete=False) as tmp_file:
            checkpoint_path = tmp_file.name
            
        try:
            # Save checkpoint
            trainer.save_checkpoint(checkpoint_path, epoch=5)
            
            # Check that the file was created
            self.assertTrue(os.path.exists(checkpoint_path))
            
            # Load checkpoint
            loaded_epoch = trainer.load_checkpoint(checkpoint_path)
            
            # Check that the epoch was loaded correctly
            self.assertEqual(loaded_epoch, 5)
            
        finally:
            # Clean up the temporary file
            if os.path.exists(checkpoint_path):
                os.remove(checkpoint_path)
                
    def test_factory_function(self):
        """Test that the factory function works correctly"""
        trainer = create_joint_trainer(self.config)
        self.assertIsInstance(trainer, JointTrainer)


if __name__ == '__main__':
    unittest.main()