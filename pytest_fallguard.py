import pytest
from pytest_embedded_idf.dut import IdfDut


@pytest.mark.supported_targets
@pytest.mark.preview_targets
@pytest.mark.generic
def test_fallguard_boot(dut: IdfDut) -> None:
    dut.expect("FallGuard boot ready")
