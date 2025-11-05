"""
    This is used for making the folder importable

    # imu_processing/__init__.py
    from .converter import IMUConverter
    from .madgwick import MadgwickFilter
    from .dead_reckoning import DeadReckoning
    from .ble_client import BLEClient
    from .visualization import PositionVisualizer, OrientationVisualizer

    __all__ = ['IMUConverter', 'MadgwickFilter', 'DeadReckoning', 
            'BLEClient', 'PositionVisualizer', 'OrientationVisualizer']

    allows clean imports from the package level:
    from imu_processing import IMUConverter, MadgwickFilter

    Also can some init logging stuff that gets run when it is first imported
    probably not need but could be important
"""
