"""asset_manager — map class -> official asset, instance pooling (STAGE 2, stub)."""


class AssetManager:
    def __init__(self, registry: dict):
        self.registry = registry

    def absolute_path(self, asset_id: str) -> str:
        import os
        a = self.registry["assets"][asset_id]
        return os.path.join(self.registry["source_root"], a["path"])

    def build_pool(self):
        raise NotImplementedError("Stage 2: pre-create max instances per class")
