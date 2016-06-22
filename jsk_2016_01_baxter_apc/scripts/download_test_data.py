#!/usr/bin/env python

from jsk_data import download_data


def main():
    PKG = 'jsk_2016_01_baxter_apc'

    # Use gdown (url) -O filename
    download_data(
        pkg_name=PKG,
        path='test_data/2016-06-22-17-33-53_apc2016-bin-boxes.bag.tgz',
        url='https://drive.google.com/uc\?id\=0BxxBA3J-CunGWHkxT296MV9ONDg',
        md5='31c955ceacef69f6fdc8afed66bbc14f',
    )

    download_data(
        pkg_name=PKG,
        path='test_data/sib_kinect2.bag.tar.gz',
        url='https://drive.google.com/uc?id=0BzBTxmVQJTrGRERod3E5S3RxdE0',
        md5='c3aaaf507b48fc7022edd51bbe819e4d',
        extract=True,
    )

    download_data(
        pkg_name=PKG,
        path='test_data/sib_right_softkinetic.bag.tar.gz',
        url='https://drive.google.com/uc?id=0BzBTxmVQJTrGTmJEaTZ5bERhMzg',
        md5='f764107a2a2fda2bb4f800c519d97dc2',
        extract=True,
    )


if __name__ == '__main__':
    main()
