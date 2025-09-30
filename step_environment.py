from world import World
from states import States
from manipulatable_object import Topping

class StepEnvironment:
    """Applies the calculated movements to the environment each iteration (robot motions, object spawns/despawns)."""
    @staticmethod
    def apply(world: World, state: States, motions: list):
        # Ensure all robot models in world have their updated joint values (so Swift shows new positions)
        # (If we updated robot.q directly in motions, Swift's env.step will already use them. If we had trajectories, we'd increment here.)
        # Spawn or remove objects if flagged (though spawning handled in state for pizzas)
        # For example, ensure toppings attached to pizzas move with them:
        for obj in world.objects:
            if isinstance(obj, Topping) and obj.attached_to_pizza:
                # update topping pose relative to pizza
                obj.mesh.pose = obj.attached_to_pizza.mesh.pose * obj.relative_pose_on_pizza
        # There might not be much to do here if other classes already updated positions.
        # This method is more relevant if motions were computed but not applied yet, or if using incremental motion.
        return
