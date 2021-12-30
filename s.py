from tensorflow.keras import models

m = models.load_model("b.h5")
m.save("b.h5")
