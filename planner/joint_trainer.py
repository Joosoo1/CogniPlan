import torch
import torch.nn as nn
import torch.optim as optim
import os
import numpy as np
from .model import PolicyNet, QNet
from .parameter import *
from mapinpaint.networks import Generator


class JointTrainer:
    """Joint training framework for end-to-end training of generator and planner"""
    
    def __init__(self, config):
        """
        Initialize the joint trainer
        
        Args:
            config (dict): Configuration dictionary containing training parameters
        """
        self.config = config
        self.device = torch.device('cuda' if config.get('use_gpu', True) and torch.cuda.is_available() else 'cpu')
        
        # Initialize networks
        self._init_networks()
        
        # Initialize optimizers
        self._init_optimizers()
        
        # Training parameters
        self.batch_size = config.get('batch_size', 32)
        self.learning_rate = config.get('learning_rate', 1e-4)
        self.gamma = config.get('gamma', 0.99)
        
    def _init_networks(self):
        """Initialize generator, policy and Q networks"""
        # Generator (map predictor)
        self.generator = Generator(self.config['generator_config'], 
                                 self.config.get('use_cuda', True)).to(self.device)
        
        # Policy and Q networks
        self.policy_net = PolicyNet(NODE_INPUT_DIM, EMBEDDING_DIM).to(self.device)
        self.q_net = QNet(NODE_INPUT_DIM, EMBEDDING_DIM).to(self.device)
        
        # Target networks for SAC
        self.target_q_net = QNet(NODE_INPUT_DIM, EMBEDDING_DIM).to(self.device)
        self.target_q_net.load_state_dict(self.q_net.state_dict())
        self.target_q_net.eval()
        
    def _init_optimizers(self):
        """Initialize optimizers for all networks"""
        # Separate optimizers for generator and policy/Q networks
        self.generator_optimizer = optim.Adam(
            self.generator.parameters(), 
            lr=self.config.get('generator_lr', 1e-4)
        )
        
        self.policy_optimizer = optim.Adam(
            self.policy_net.parameters(), 
            lr=self.config.get('policy_lr', 1e-4)
        )
        
        self.q_optimizer = optim.Adam(
            self.q_net.parameters(), 
            lr=self.config.get('q_lr', 1e-4)
        )
        
    def forward_pass(self, x, mask, onehot):
        """
        Forward pass through the entire differentiable pipeline
        
        Args:
            x: Input tensor (map data)
            mask: Mask tensor
            onehot: One-hot encoded tensor
            
        Returns:
            dict: Dictionary containing outputs from all stages
        """
        # 1. Generator (map prediction)
        generated_map = self.generator(x, mask, onehot)
        inpainted_result = generated_map * mask + x * (1. - mask)
        
        # 2. Convert predicted map to graph representation for planning
        graph_representation = self.convert_map_to_graph(inpainted_result)
        
        # 3. Planning (using predicted map)
        # Get policy output (action probabilities)
        logp = self.policy_net(
            graph_representation['node_inputs'],
            graph_representation['node_padding_mask'],
            graph_representation['edge_mask'],
            graph_representation['current_index'],
            graph_representation['current_edge'],
            graph_representation['edge_padding_mask']
        )
        
        # Get Q-values
        q_values = self.q_net(
            graph_representation['node_inputs'],
            graph_representation['node_padding_mask'],
            graph_representation['edge_mask'],
            graph_representation['current_index'],
            graph_representation['current_edge'],
            graph_representation['edge_padding_mask']
        )
        
        return {
            'generated_map': generated_map,
            'inpainted_result': inpainted_result,
            'policy_output': logp,
            'q_values': q_values,
            'graph_representation': graph_representation
        }
        
    def compute_planning_loss(self, node_inputs, node_padding_mask, edge_mask, 
                             current_index, current_edge, edge_padding_mask, actions, rewards):
        """
        Compute planning loss using policy and Q networks
        
        Args:
            node_inputs: Graph node features
            node_padding_mask: Node padding mask
            edge_mask: Edge mask
            current_index: Current node index
            current_edge: Current edge information
            edge_padding_mask: Edge padding mask
            actions: Action values
            rewards: Reward values
            
        Returns:
            dict: Dictionary containing planning losses
        """
        # Get policy output (action probabilities)
        logp = self.policy_net(node_inputs, node_padding_mask, edge_mask, 
                              current_index, current_edge, edge_padding_mask)
        
        # Get Q-values
        q_values = self.q_net(node_inputs, node_padding_mask, edge_mask, 
                             current_index, current_edge, edge_padding_mask)
        
        # Compute policy loss (using REINFORCE or SAC)
        policy_loss = torch.sum(
            (logp.exp().unsqueeze(2) * (-q_values).detach()),
            dim=1).mean()
        
        # Compute Q-value loss
        q_values_selected = torch.gather(q_values, 1, actions)
        target_q = rewards  # Simplified - would normally include next state values
        q_loss = nn.MSELoss()(q_values_selected, target_q.detach()).mean()
        
        return {
            'policy_loss': policy_loss,
            'q_loss': q_loss
        }
        
    def compute_loss(self, predictions, targets, rewards, actions, 
                    planning_inputs=None):
        """
        Compute joint loss for generator and planner
        
        Args:
            predictions: Model predictions
            targets: Target values
            rewards: Reward values
            actions: Action values
            planning_inputs: Inputs for planning network (optional)
            
        Returns:
            dict: Dictionary containing all loss components
        """
        # Generator loss (reconstruction loss)
        generator_loss = nn.MSELoss()(predictions['inpainted_result'], targets)
        
        # Planning loss (if planning inputs are provided)
        if planning_inputs is not None:
            planning_losses = self.compute_planning_loss(
                planning_inputs['node_inputs'],
                planning_inputs['node_padding_mask'],
                planning_inputs['edge_mask'],
                planning_inputs['current_index'],
                planning_inputs['current_edge'],
                planning_inputs['edge_padding_mask'],
                actions, rewards
            )
            # Total loss combines generator and planning losses
            total_loss = generator_loss + planning_losses['policy_loss'] + planning_losses['q_loss']
            
            return {
                'generator_loss': generator_loss,
                'policy_loss': planning_losses['policy_loss'],
                'q_loss': planning_losses['q_loss'],
                'total_loss': total_loss
            }
        else:
            # Only generator loss if no planning inputs
            return {
                'generator_loss': generator_loss,
                'total_loss': generator_loss
            }
        
    def train_step(self, batch_data):
        """
        Perform one training step on a batch of data
        
        Args:
            batch_data: Batch of training data containing:
                - x: Input maps
                - mask: Mask tensors
                - onehot: One-hot encoded tensors
                - targets: Target maps
                - rewards: Reward values
                - actions: Action values
                - planning_inputs: Optional inputs for planning network
                
        Returns:
            dict: Training metrics
        """
        # Extract batch data
        x, mask, onehot, targets = batch_data[:4]
        rewards, actions = batch_data[4:6]
        planning_inputs = batch_data[6] if len(batch_data) > 6 else None
        
        # Move to device
        x = x.to(self.device)
        mask = mask.to(self.device)
        onehot = onehot.to(self.device)
        targets = targets.to(self.device)
        rewards = rewards.to(self.device)
        actions = actions.to(self.device)
        
        if planning_inputs is not None:
            for key in planning_inputs:
                planning_inputs[key] = planning_inputs[key].to(self.device)
        
        # Forward pass
        predictions = self.forward_pass(x, mask, onehot)
        
        # Compute loss
        losses = self.compute_loss(predictions, targets, rewards, actions, planning_inputs)
        
        # Generator backward pass
        self.generator_optimizer.zero_grad()
        losses['total_loss'].backward()
        self.generator_optimizer.step()
        
        # Update policy and Q networks if planning inputs are provided
        if planning_inputs is not None:
            self.policy_optimizer.zero_grad()
            losses['policy_loss'].backward()
            self.policy_optimizer.step()
            
            self.q_optimizer.zero_grad()
            losses['q_loss'].backward()
            self.q_optimizer.step()
        
        # Return metrics
        metrics = {
            'loss': losses['total_loss'].item(),
            'generator_loss': losses['generator_loss'].item()
        }
        
        if planning_inputs is not None:
            metrics.update({
                'policy_loss': losses['policy_loss'].item(),
                'q_loss': losses['q_loss'].item()
            })
        
        return metrics
        
    def save_checkpoint(self, path, epoch):
        """
        Save model checkpoint
        
        Args:
            path (str): Path to save checkpoint
            epoch (int): Current epoch
        """
        checkpoint = {
            'epoch': epoch,
            'generator_state_dict': self.generator.state_dict(),
            'policy_net_state_dict': self.policy_net.state_dict(),
            'q_net_state_dict': self.q_net.state_dict(),
            'generator_optimizer_state_dict': self.generator_optimizer.state_dict(),
            'policy_optimizer_state_dict': self.policy_optimizer.state_dict(),
            'q_optimizer_state_dict': self.q_optimizer.state_dict()
        }
        
        torch.save(checkpoint, path)
        
    def load_checkpoint(self, path):
        """
        Load model checkpoint
        
        Args:
            path (str): Path to checkpoint
        """
        checkpoint = torch.load(path, map_location=self.device)
        
        self.generator.load_state_dict(checkpoint['generator_state_dict'])
        self.policy_net.load_state_dict(checkpoint['policy_net_state_dict'])
        self.q_net.load_state_dict(checkpoint['q_net_state_dict'])
        
        self.generator_optimizer.load_state_dict(checkpoint['generator_optimizer_state_dict'])
        self.policy_optimizer.load_state_dict(checkpoint['policy_optimizer_state_dict'])
        self.q_optimizer.load_state_dict(checkpoint['q_optimizer_state_dict'])
        
        return checkpoint.get('epoch', 0)
        
    def convert_map_to_graph(self, predicted_map):
        """
        Convert predicted map to graph representation for planning
        
        Args:
            predicted_map: Predicted map from generator
            
        Returns:
            dict: Graph representation with node features, edges, etc.
        """
        # This is a placeholder for the actual implementation
        # In a full implementation, this would:
        # 1. Extract frontier points from the predicted map
        # 2. Create graph nodes from frontier points and robot position
        # 3. Compute node features (distance, utility, etc.)
        # 4. Create edges between nodes based on connectivity
        # 5. Return the graph representation suitable for PolicyNet
        
        # For now, return a placeholder with correct dimensions
        batch_size = predicted_map.size(0)
        num_nodes = 100
        k_size = 10  # Number of neighboring nodes
        
        # Create dummy graph representation with correct dimensions
        node_inputs = torch.randn(batch_size, num_nodes, NODE_INPUT_DIM).to(self.device)
        node_padding_mask = torch.zeros(batch_size, 1, num_nodes, dtype=torch.int16).to(self.device)
        edge_mask = torch.ones(batch_size, num_nodes, num_nodes).to(self.device)
        current_index = torch.zeros(batch_size, 1, dtype=torch.long).to(self.device)  # Shape: [batch_size, 1]
        
        # Create current_edge with the correct shape [batch_size, k_size]
        current_edge = torch.arange(k_size).unsqueeze(0).repeat(batch_size, 1).to(self.device)
        
        # Create edge_padding_mask with the correct shape [batch_size, k_size]
        edge_padding_mask = torch.zeros(batch_size, k_size, dtype=torch.int16).to(self.device)
        
        return {
            'node_inputs': node_inputs,
            'node_padding_mask': node_padding_mask,
            'edge_mask': edge_mask,
            'current_index': current_index,
            'current_edge': current_edge,
            'edge_padding_mask': edge_padding_mask
        }


# Example usage function
def create_joint_trainer(config):
    """
    Factory function to create a JointTrainer instance
    
    Args:
        config (dict): Configuration dictionary
        
    Returns:
        JointTrainer: Initialized trainer instance
    """
    return JointTrainer(config)


    def convert_map_to_graph(self, predicted_map):
        """
        Convert predicted map to graph representation for planning
        
        Args:
            predicted_map: Predicted map from generator
            
        Returns:
            dict: Graph representation with node features, edges, etc.
        """
        # This is a placeholder for the actual implementation
        # In a full implementation, this would:
        # 1. Extract frontier points from the predicted map
        # 2. Create graph nodes from frontier points and robot position
        # 3. Compute node features (distance, utility, etc.)
        # 4. Create edges between nodes based on connectivity
        # 5. Return the graph representation suitable for PolicyNet
        
        # For now, return a placeholder with correct dimensions
        batch_size = predicted_map.size(0)
        num_nodes = 100
        k_size = 10  # Number of neighboring nodes
        
        # Create dummy graph representation with correct dimensions
        node_inputs = torch.randn(batch_size, num_nodes, NODE_INPUT_DIM).to(self.device)
        node_padding_mask = torch.zeros(batch_size, 1, num_nodes, dtype=torch.int16).to(self.device)
        edge_mask = torch.ones(batch_size, num_nodes, num_nodes).to(self.device)
        current_index = torch.zeros(batch_size, 1, dtype=torch.long).to(self.device)  # Shape: [batch_size, 1]
        
        # Create current_edge with the correct shape [batch_size, k_size]
        current_edge = torch.arange(k_size).unsqueeze(0).repeat(batch_size, 1).to(self.device)
        
        # Create edge_padding_mask with the correct shape [batch_size, k_size]
        edge_padding_mask = torch.zeros(batch_size, k_size, dtype=torch.int16).to(self.device)
        
        return {
            'node_inputs': node_inputs,
            'node_padding_mask': node_padding_mask,
            'edge_mask': edge_mask,
            'current_index': current_index,
            'current_edge': current_edge,
            'edge_padding_mask': edge_padding_mask
        }