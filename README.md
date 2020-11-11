# 10gbe

## Hardware

KC705 + Si5xx board that generates transceiver clock for 10GbE

Note with current setup program an external clock generator to 156.25MHz and connect to J15-16 SMAs of KC705

I used an SFP+ fiber transceiver on the KC705 -> LC fiber -> 10GbE pcie network card on a desktop (with another SFP+ fiber transceiver on the other side of the LC cable). Looks like Copper can be used, but I haven't really tried.

## Xilinx IP

ten_gig_eth_pcs_pma_0.xci @ 2018.3.1 vivado

## LiteEth

https://github.com/jersey99/liteeth/tree/10gbe-test-setup

## Information

```
python 10g.py --with-ethernet --cpu-type None --csr-csv csr.csv --build --load
```

1. Note that sys_clk domain for xgmii is renamed to 156.25e6 domain. This keeps testing logic simple for my purposes.
2. In the past I hadn't tested with a full litex top. I was used to generating a verilog file and including it in another larger project. Long story short, I couldn't find a way to add the false_path constraints in the right order after the create_clock. Once you synthesize with the above command you will see what I mean.
3. Once this is up and running wireshark is your friend. You should see a stream of udp packets (once a second). I have used the same style testing on this reaching high 9+GbE a year ago. These can also be read with a python script.
4. Ping replies are malformed and need work.
