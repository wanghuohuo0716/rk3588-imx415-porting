#ifndef FIREFLY_PCA953X_HPD_H
#define FIREFLY_PCA953X_HPD_H
 

struct firefly_pca953x_hpd_info {
    struct platform_device  *pdev;
    int                     det_gpio;
    int                     det_irq;
    int                     flag;
};

int firefly_pca953x_is_inserted(struct platform_device *pdev, bool *is_inserted);

#endif
