import time

from litex.tools.litex_client import RemoteClient

def main(args):
    wb = RemoteClient()
    wb.open()
    # print('reset before: ', wb.regs.xgmii_reset.read())
    # wb.regs.xgmii_reset.write(1)
    # print('reset after: ', wb.regs.xgmii_reset.read())
    # time.sleep(5)
    # print('reset after: ', wb.regs.xgmii_reset.read())
    # wb.regs.xgmii_reset.write(0)
    # print('reset after: ', wb.regs.xgmii_reset.read())


    print(f'pcs status: {hex(wb.regs.xgmii_pcs_status.read())}')
    print('pcs reset: ', wb.regs.xgmii_pcs_config.write(1))

    print(f'pcs status: {hex(wb.regs.xgmii_pcs_status.read())}')

    for i in range(1):
        print(wb.regs.f_sample_value.read()/1e6)
        time.sleep(1)
    wb.close()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    main(args)
