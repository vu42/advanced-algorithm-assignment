#!/usr/bin/env python3
"""
Generate diagrams for Unified Examples in test2.md.

Each diagram illustrates a specific phase of the BA* robot planner:
1. Phase 1 (Movement) - Rotation and translation
2. Phase 1 (Tiling) - Tile stamping 
3. Phase 2 (Stuck) - Critical point detection
4. Phase 2 (List Check) - Backtracking candidate eligibility
5. Phase 3 (Selection + Execution) - Path planning and execution
"""

from __future__ import annotations

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch, Circle, Rectangle, Wedge
from pathlib import Path


# Configuration
ROBOT_RADIUS = 5
OUTPUT_DIR = Path(__file__).parent.parent / "outputs"


def setup_figure(figsize=(8, 6)):
    """Create a clean figure with white background."""
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_facecolor('white')
    ax.set_aspect('equal')
    return fig, ax


def draw_robot(ax, x, y, theta, radius=ROBOT_RADIUS, color='#3498db', alpha=0.8, label=None):
    """Draw a robot as a circle with heading indicator."""
    # Robot body
    robot = Circle((x, y), radius, facecolor=color, edgecolor='#2c3e50', 
                   linewidth=2, alpha=alpha, label=label)
    ax.add_patch(robot)
    
    # Heading indicator (arrow from center)
    dx = radius * 0.8 * np.cos(theta)
    dy = radius * 0.8 * np.sin(theta)
    ax.arrow(x, y, dx, dy, head_width=1.5, head_length=1, fc='#2c3e50', ec='#2c3e50')
    
    return robot


def draw_tile(ax, x, y, size=10, color='#2ecc71', alpha=0.4, edgecolor='#27ae60', label=None):
    """Draw a tile as a square centered at (x, y)."""
    rect = Rectangle((x - size/2, y - size/2), size, size,
                      facecolor=color, edgecolor=edgecolor, linewidth=1.5, 
                      alpha=alpha, label=label)
    ax.add_patch(rect)
    return rect


def save_figure(fig, name):
    """Save figure to output directory."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUTPUT_DIR / f"{name}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"Saved: {path}")


# =============================================================================
# DIAGRAM 1: Phase 1 (Movement) - Rotation and Translation
# =============================================================================
def fig_phase1_movement():
    """Robot at (12,75) rotates North, then moves to (12,80)."""
    fig, ax = setup_figure(figsize=(10, 8))
    
    # Grid area
    ax.set_xlim(-5, 40)
    ax.set_ylim(55, 100)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Initial position - facing East (theta=0)
    draw_robot(ax, 12, 75, theta=0, color='#bdc3c7', alpha=0.5, label='Initial (12,75) θ=0')
    
    # After rotation - facing North (theta=π/2)
    draw_robot(ax, 12, 75, theta=np.pi/2, color='#3498db', alpha=0.6, label='After rotation θ=π/2')
    
    # Draw curved arrow for rotation
    arc = patches.Arc((12, 75), 12, 12, angle=0, theta1=0, theta2=90,
                       color='#e74c3c', linewidth=2, linestyle='-')
    ax.add_patch(arc)
    ax.annotate('α = π/2', xy=(20, 80), fontsize=11, color='#e74c3c', fontweight='bold')
    
    # Final position after translation
    draw_robot(ax, 12, 80, theta=np.pi/2, color='#27ae60', alpha=0.8, label='Final (12,80)')
    
    # Translation arrow
    ax.annotate('', xy=(12, 80), xytext=(12, 75),
                arrowprops=dict(arrowstyle='->', color='#9b59b6', lw=2.5))
    ax.annotate('d = 5', xy=(4, 77.5), fontsize=11, color='#9b59b6', fontweight='bold')
    
    # Annotations
    ax.set_title('Phase 1 (Movement)\nRotation + Translation', fontsize=14, fontweight='bold')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(loc='upper right', framealpha=0.9)
    
    # Add equations box
    textstr = r'$q_{start} = (12, 75, 0)$' + '\n' + \
              r'Rotate: $\alpha = \pi/2$' + '\n' + \
              r'Move: $d = 5$' + '\n' + \
              r'$q_{final} = (12, 80, \pi/2)$'
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)
    
    save_figure(fig, 'fig_phase1_movement')


# =============================================================================
# DIAGRAM 2: Phase 1 (Tiling) - Creating tiles as robot moves
# =============================================================================
def fig_phase1_tiling():
    """Show tiles stamped at (12,75) and (12,80)."""
    fig, ax = setup_figure(figsize=(10, 8))
    
    # Grid area
    ax.set_xlim(-5, 40)
    ax.set_ylim(55, 100)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Tile 1 at (12, 75)
    draw_tile(ax, 12, 75, size=10, color='#3498db', alpha=0.4, label='Tile s₁ = (12, 75, 10)')
    ax.plot(12, 75, 'ko', markersize=5)
    ax.annotate('(12, 75)', xy=(12, 75), xytext=(20, 72), fontsize=10,
                arrowprops=dict(arrowstyle='->', color='gray'))
    
    # Tile 2 at (12, 80) - overlapping
    draw_tile(ax, 12, 80, size=10, color='#e74c3c', alpha=0.4, label='Tile s₂ = (12, 80, 10)')
    ax.plot(12, 80, 'ko', markersize=5)
    ax.annotate('(12, 80)', xy=(12, 80), xytext=(20, 83), fontsize=10,
                arrowprops=dict(arrowstyle='->', color='gray'))
    
    # Robot path
    ax.plot([12, 12], [75, 80], 'g--', linewidth=2, label='Robot path')
    
    # Draw robot at final position
    draw_robot(ax, 12, 80, theta=np.pi/2, color='#27ae60', alpha=0.7)
    
    ax.set_title('Phase 1 (Tiling)\nTiles stamped as robot moves', fontsize=14, fontweight='bold')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(loc='upper right', framealpha=0.9)
    
    # Map state box
    textstr = r'Map $M$:' + '\n' + \
              r'$s_1 = (12, 75, 10)$' + '\n' + \
              r'$s_2 = (12, 80, 10)$'
    props = dict(boxstyle='round', facecolor='lightgreen', alpha=0.8)
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)
    
    save_figure(fig, 'fig_phase1_tiling')


# =============================================================================
# DIAGRAM 3: Phase 2 (Stuck) - Critical point detection
# =============================================================================
def fig_phase2_stuck():
    """Robot at critical point (88,1) with blocked neighbors."""
    fig, ax = setup_figure(figsize=(10, 8))
    
    # Grid area focused on critical point
    ax.set_xlim(65, 110)
    ax.set_ylim(-20, 25)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Current tile at (88, 1)
    draw_tile(ax, 88, 1, size=10, color='#e74c3c', alpha=0.5, edgecolor='#c0392b',
              label='Current tile s = (88, 1)')
    
    # N4 neighbors - all blocked
    neighbors = [
        (88, 11, 'N', '↑'),   # North
        (88, -9, 'S', '↓'),   # South
        (98, 1, 'E', '→'),    # East
        (78, 1, 'W', '←'),    # West
    ]
    
    for nx, ny, direction, arrow in neighbors:
        # Draw blocked neighbor
        draw_tile(ax, nx, ny, size=10, color='#7f8c8d', alpha=0.6, edgecolor='#2c3e50')
        ax.text(nx, ny, '✗', fontsize=16, ha='center', va='center', color='#c0392b', fontweight='bold')
    
    # Robot at critical point
    draw_robot(ax, 88, 1, theta=0, color='#e74c3c', alpha=0.8, label='Robot (stuck)')
    
    # Add legend for blocked
    blocked_patch = patches.Patch(facecolor='#7f8c8d', edgecolor='#2c3e50', 
                                   alpha=0.6, label='Blocked neighbors N₄(s)')
    ax.add_patch(patches.Rectangle((0, 0), 0, 0, visible=False))
    
    ax.set_title('Phase 2 (Stuck)\nCritical Point Detection', fontsize=14, fontweight='bold')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(handles=[blocked_patch] + ax.get_legend_handles_labels()[0], 
              loc='upper right', framealpha=0.9)
    
    # Status box
    textstr = 'Critical Point Check:\n' + \
              r'$N_4(s)$ = {N, S, E, W}' + '\n' + \
              'All 4 blocked → STUCK\n' + \
              r'isCritical(88,1) = True'
    props = dict(boxstyle='round', facecolor='#ffcccc', alpha=0.9)
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)
    
    save_figure(fig, 'fig_phase2_stuck')


# =============================================================================
# DIAGRAM 4: Phase 2 (List Check) - Backtracking candidate eligibility
# =============================================================================
def fig_phase2_list_check():
    """Show tile (52,1) as a backtracking candidate."""
    fig, ax = setup_figure(figsize=(10, 8))
    
    # Grid area
    ax.set_xlim(30, 75)
    ax.set_ylim(-15, 25)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Candidate tile at (52, 1)
    draw_tile(ax, 52, 1, size=10, color='#2ecc71', alpha=0.5, edgecolor='#27ae60',
              label='Candidate s = (52, 1)')
    
    # Some neighbors - mixed free/blocked to show corner pattern
    # Free neighbor (West)
    draw_tile(ax, 42, 1, size=10, color='#3498db', alpha=0.3, edgecolor='#2980b9')
    ax.text(42, 1, 'Free', fontsize=9, ha='center', va='center', color='#2980b9')
    
    # Blocked neighbor (South) - creates corner pattern  
    draw_tile(ax, 52, -9, size=10, color='#7f8c8d', alpha=0.5, edgecolor='#2c3e50')
    ax.text(52, -9, 'Blocked', fontsize=9, ha='center', va='center', color='#2c3e50')
    
    # Free neighbor (East)
    draw_tile(ax, 62, 1, size=10, color='#3498db', alpha=0.3, edgecolor='#2980b9')
    ax.text(62, 1, 'Free', fontsize=9, ha='center', va='center', color='#2980b9')
    
    # Free neighbor (North)
    draw_tile(ax, 52, 11, size=10, color='#3498db', alpha=0.3, edgecolor='#2980b9')
    ax.text(52, 11, 'Free', fontsize=9, ha='center', va='center', color='#2980b9')
    
    # Mark center
    ax.plot(52, 1, 'g*', markersize=15, label='s = (52, 1)')
    
    ax.set_title('Phase 2 (List Check)\nBacktracking Candidate Eligibility', fontsize=14, fontweight='bold')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(loc='upper right', framealpha=0.9)
    
    # Explanation box
    textstr = 'Corner-like condition:\n' + \
              r'Adjacent pair $(s_i, s_j)$:' + '\n' + \
              '• West: Free\n' + \
              '• South: Blocked\n' + \
              r'$b(s_W, s_S) = 1 \Rightarrow \mu(s) \geq 1$' + '\n' + \
              '→ Add to list L'
    props = dict(boxstyle='round', facecolor='#e8f8e8', alpha=0.9)
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=props)
    
    save_figure(fig, 'fig_phase2_list_check')


# =============================================================================
# DIAGRAM 5: Phase 3 (Selection) - Candidate selection
# =============================================================================
def fig_phase3_selection():
    """Show candidate list L and selection of s_sp from s_cp."""
    fig, ax = setup_figure(figsize=(12, 8))
    
    # Grid area covering the critical point and showing candidates spread
    ax.set_xlim(30, 105)
    ax.set_ylim(-15, 30)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Critical point (88, 1) - where robot is stuck
    draw_tile(ax, 88, 1, size=10, color='#e74c3c', alpha=0.5, edgecolor='#c0392b',
              label=r'Critical Point $s_{cp}$ = (88, 1)')
    draw_robot(ax, 88, 1, theta=0, color='#e74c3c', alpha=0.8)
    
    # Show some candidate tiles from L (simulated distribution)
    # These represent the 32 candidates mentioned in summary.json
    candidate_positions = [
        (52, 1), (42, 1), (62, 1), (72, 1),  # along x-axis
        (52, 11), (62, 11), (72, 11),         # row above
        (42, -9), (52, -9), (62, -9),         # row below
        (35, 1), (35, 11), (45, 11),          # left side
        (45, -9), (55, -9), (65, -9),         # more candidates
    ]
    
    for i, (cx, cy) in enumerate(candidate_positions):
        if (cx, cy) == (52, 1):
            continue  # Skip the selected one, we'll draw it specially
        draw_tile(ax, cx, cy, size=8, color='#f39c12', alpha=0.4, edgecolor='#e67e22')
    
    # Add legend for candidates
    candidate_patch = patches.Patch(facecolor='#f39c12', edgecolor='#e67e22', 
                                     alpha=0.4, label=f'Candidates in L (|L| = 32)')
    
    # Selected starting point (52, 1) - highlighted
    draw_tile(ax, 52, 1, size=10, color='#2ecc71', alpha=0.6, edgecolor='#27ae60',
              label=r'Selected $s_{sp}$ = (52, 1)')
    ax.plot(52, 1, 'g*', markersize=25, zorder=10)
    
    # Draw arrow from s_cp to s_sp showing selection
    ax.annotate('', xy=(52, 1), xytext=(88, 1),
                arrowprops=dict(arrowstyle='->', color='#9b59b6', lw=2.5,
                               linestyle='--', connectionstyle='arc3,rad=0.1'))
    ax.annotate('min cost path', xy=(70, 6), fontsize=10, color='#9b59b6', 
                fontweight='bold', ha='center')
    
    ax.set_title('Phase 3 (Selection)\nChoosing Next Starting Point from Candidates', fontsize=14, fontweight='bold')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    
    handles, labels = ax.get_legend_handles_labels()
    handles.insert(0, candidate_patch)
    ax.legend(handles=handles, loc='upper right', framealpha=0.9)
    
    # Selection info box
    textstr = 'Selection Process:\n' + \
              '• Candidate count: |L| = 32\n' + \
              '• For each candidate, compute\n' + \
              r'  $J(s_{cp}, s) = $ A* path cost' + '\n' + \
              r'• $s_{sp} = \arg\min_{s \in L} J(s_{cp}, s)$' + '\n\n' + \
              'Result:\n' + \
              '• A* path length: 37 steps\n' + \
              '• Smoothed path: 2 waypoints'
    props = dict(boxstyle='round', facecolor='#fff3e0', alpha=0.9)
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=9,
            verticalalignment='top', bbox=props)
    
    save_figure(fig, 'fig_phase3_selection')


# =============================================================================
# DIAGRAM 6: Phase 3 (Execution) - Path planning and execution
# =============================================================================
def fig_phase3_execution():
    """Show path from critical point (88,1) to target (52,1)."""
    fig, ax = setup_figure(figsize=(12, 6))
    
    # Grid area covering both points
    ax.set_xlim(35, 105)
    ax.set_ylim(-15, 20)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Critical point (88, 1)
    draw_tile(ax, 88, 1, size=10, color='#e74c3c', alpha=0.4, edgecolor='#c0392b',
              label=r'Critical Point $s_{cp}$ = (88, 1)')
    draw_robot(ax, 88, 1, theta=0, color='#e74c3c', alpha=0.8)
    
    # Target point (52, 1)
    draw_tile(ax, 52, 1, size=10, color='#2ecc71', alpha=0.4, edgecolor='#27ae60',
              label=r'Selected Point $s_{sp}$ = (52, 1)')
    ax.plot(52, 1, 'g*', markersize=20)
    
    # Path arrow
    ax.annotate('', xy=(52, 1), xytext=(88, 1),
                arrowprops=dict(arrowstyle='->', color='#9b59b6', lw=3,
                               connectionstyle='arc3,rad=0'))
    
    # Distance annotation
    ax.annotate('d = 36', xy=(70, 4), fontsize=12, color='#9b59b6', fontweight='bold')
    
    # Heading annotation
    ax.annotate(r'$\beta = \pi$ (West)', xy=(70, -3), fontsize=11, color='#3498db', fontweight='bold')
    
    ax.set_title('Phase 3 (Execution)\nBacktracking Path Computation', fontsize=14, fontweight='bold')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend(loc='upper right', framealpha=0.9)
    
    # Control commands box
    textstr = 'Control Commands:\n' + \
              r'$\beta = \arctan(\frac{1-1}{52-88}) = \pi$' + '\n' + \
              r'$\alpha = \beta - \theta = \pi - \theta$' + '\n' + \
              r'$d = \sqrt{(52-88)^2 + (1-1)^2} = 36$'
    props = dict(boxstyle='round', facecolor='#f0e6ff', alpha=0.9)
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=9,
            verticalalignment='top', bbox=props)
    
    save_figure(fig, 'fig_phase3_execution')


def main():
    """Generate all unified example diagrams."""
    print("Generating Unified Example Diagrams...")
    print(f"Output directory: {OUTPUT_DIR}")
    print("-" * 50)
    
    fig_phase1_movement()
    fig_phase1_tiling()
    fig_phase2_stuck()
    fig_phase2_list_check()
    fig_phase3_selection()
    fig_phase3_execution()
    
    print("-" * 50)
    print("Done! Generated 6 diagrams.")


if __name__ == "__main__":
    main()
