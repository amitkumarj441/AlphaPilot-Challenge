import numpy as np

from .formulas import non_maximum_suppression

class PriorMap(object):
    """Handles prior boxes for a given feature map.

    # Arguments / Attributes
        source_layer_name
        image_size: Tuple with spatial size of model input.
        map_size
        variances
        aspect_ratios: List of aspect ratios for the prior boxes at each
            location.
        shift: List of tuples for the displacement of the prior boxes
            relative to ther location. Each tuple contains an value between
            -1.0 and 1.0 for x and y direction.
        clip: Boolean, whether the boxes should be cropped to do not exceed
            the borders of the input image.
        step
        minmax_size: List of tuples with s_min and s_max values (see paper).
        special_ssd_box: Boolean, wether or not the extra box for aspect
            ratio 1 is used.

    # Notes
        The compute_priors methode has to be called to get usable prior boxes.
    """

    def __init__(
        self,
        source_layer_name,
        image_size,
        map_size,
        minmax_size=None,
        variances=[0.1, 0.1, 0.2, 0.2],
        aspect_ratios=[1],
        shift=None,
        clip=False,
        step=None,
        special_ssd_box=False,
    ):

        self.__dict__.update(locals())

    def compute_priors(self):
        image_h, image_w = image_size = self.image_size
        map_h, map_w = map_size = self.map_size
        min_size, max_size = self.minmax_size

        # define centers of prior boxes
        if self.step is None:
            step_x = image_w / map_w
            step_y = image_h / map_h
            assert (
                step_x % 1 == 0 and step_y % 1 == 0
            ), "map size %s not constiten with input height %s" % (map_size, image_size)
        else:
            step_x = step_y = self.step

        linx = np.array([(0.5 + i) for i in range(map_w)]) * step_x
        liny = np.array([(0.5 + i) for i in range(map_h)]) * step_y
        box_xy = np.array(np.meshgrid(linx, liny)).reshape(2, -1).T

        if self.shift is None:
            shift = [(0.0, 0.0)] * len(self.aspect_ratios)
        else:
            shift = self.shift

        box_wh = []
        box_shift = []
        for i in range(len(self.aspect_ratios)):
            ar = self.aspect_ratios[i]
            box_wh.append([min_size * np.sqrt(ar), min_size / np.sqrt(ar)])
            box_shift.append(shift[i])
            if ar == 1 and self.special_ssd_box:  # special SSD box
                box_wh.append(
                    [np.sqrt(min_size * max_size), np.sqrt(min_size * max_size)]
                )
                box_shift.append((0.0, 0.0))
        box_wh = np.asarray(box_wh)

        box_shift = np.asarray(box_shift)
        box_shift = np.clip(box_shift, -1.0, 1.0)
        box_shift = box_shift * 0.5 * np.array([step_x, step_y])  # percent to pixels

        # values for individual prior boxes
        priors_shift = np.tile(box_shift, (len(box_xy), 1))
        priors_xy = np.repeat(box_xy, len(box_wh), axis=0) + priors_shift
        priors_wh = np.tile(box_wh, (len(box_xy), 1))

        priors_min_xy = priors_xy - priors_wh / 2.0
        priors_max_xy = priors_xy + priors_wh / 2.0

        if self.clip:
            priors_min_xy[:, 0] = np.clip(priors_min_xy[:, 0], 0, image_w)
            priors_min_xy[:, 1] = np.clip(priors_min_xy[:, 1], 0, image_h)
            priors_max_xy[:, 0] = np.clip(priors_max_xy[:, 0], 0, image_w)
            priors_max_xy[:, 1] = np.clip(priors_max_xy[:, 1], 0, image_h)

        priors_variances = np.tile(self.variances, (len(priors_xy), 1))

        self.box_xy = box_xy
        self.box_wh = box_wh
        self.box_shfit = box_shift

        self.priors_xy = priors_xy
        self.priors_wh = priors_wh
        self.priors_min_xy = priors_min_xy
        self.priors_max_xy = priors_max_xy
        self.priors_variances = priors_variances
        self.priors = np.concatenate(
            [priors_min_xy, priors_max_xy, priors_variances], axis=1
        )


class PriorUtil(object):
    """Utility for SSD prior boxes.
    """

    def __init__(
        self,
        model,
        aspect_ratios=None,
        shifts=None,
        minmax_sizes=None,
        steps=None,
        scale=None,
        clips=None,
        special_ssd_boxes=None,
        ssd_assignment=None,
    ):

        source_layers_names = [l.name.split("/")[0] for l in model.source_layers]
        self.source_layers_names = source_layers_names

        self.model = model
        self.image_size = model.input_shape[1:3]

        num_maps = len(source_layers_names)

        # take parameters from model definition if they exist there
        if aspect_ratios is None:
            if hasattr(model, "aspect_ratios"):
                aspect_ratios = model.aspect_ratios
            else:
                aspect_ratios = [[1]] * num_maps

        if shifts is None:
            if hasattr(model, "shifts"):
                shifts = model.shifts
            else:
                shifts = [None] * num_maps

        if minmax_sizes is None:
            if hasattr(model, "minmax_sizes"):
                minmax_sizes = model.minmax_sizes
            else:
                # as in equation (4)
                min_dim = np.min(self.image_size)
                min_ratio = 10  # 15
                max_ratio = 100  # 90
                s = np.linspace(min_ratio, max_ratio, num_maps + 1) * min_dim / 100.0
                minmax_sizes = [
                    (round(s[i]), round(s[i + 1])) for i in range(len(s) - 1)
                ]

        if scale is None:
            if hasattr(model, "scale"):
                scale = model.scale
            else:
                scale = 1.0
        minmax_sizes = np.array(minmax_sizes) * scale

        if steps is None:
            if hasattr(model, "steps"):
                steps = model.steps
            else:
                steps = [None] * num_maps

        if clips is None:
            if hasattr(model, "clips"):
                clips = model.clips
            else:
                clips = False
        if type(clips) == bool:
            clips = [clips] * num_maps

        if special_ssd_boxes is None:
            if hasattr(model, "special_ssd_boxes"):
                special_ssd_boxes = model.special_ssd_boxes
            else:
                special_ssd_boxes = False
        if type(special_ssd_boxes) == bool:
            special_ssd_boxes = [special_ssd_boxes] * num_maps

        if ssd_assignment is None:
            if hasattr(model, "ssd_assignment"):
                ssd_assignment = model.ssd_assignment
            else:
                ssd_assignment = True
        self.ssd_assignment = ssd_assignment

        self.prior_maps = []
        for i in range(num_maps):
            layer = model.get_layer(source_layers_names[i])
            map_h, map_w = map_size = layer.output_shape[1:3]
            m = PriorMap(
                source_layer_name=source_layers_names[i],
                image_size=self.image_size,
                map_size=map_size,
                minmax_size=minmax_sizes[i],
                variances=[0.1, 0.1, 0.2, 0.2],
                aspect_ratios=aspect_ratios[i],
                shift=shifts[i],
                step=steps[i],
                special_ssd_box=special_ssd_boxes[i],
                clip=clips[i],
            )
            self.prior_maps.append(m)
        self.update_priors()

        self.nms_top_k = 400
        self.nms_thresh = 0.45

    def update_priors(self):
        priors_xy = []
        priors_wh = []
        priors_min_xy = []
        priors_max_xy = []
        priors_variances = []
        priors = []

        map_offsets = [0]
        for i in range(len(self.prior_maps)):
            m = self.prior_maps[i]

            # compute prior boxes
            m.compute_priors()

            # collect prior data
            priors_xy.append(m.priors_xy)
            priors_wh.append(m.priors_wh)
            priors_min_xy.append(m.priors_min_xy)
            priors_max_xy.append(m.priors_max_xy)
            priors_variances.append(m.priors_variances)
            priors.append(m.priors)
            map_offsets.append(map_offsets[-1] + len(m.priors))

        self.priors_xy = np.concatenate(priors_xy, axis=0)
        self.priors_wh = np.concatenate(priors_wh, axis=0)
        self.priors_min_xy = np.concatenate(priors_min_xy, axis=0)
        self.priors_max_xy = np.concatenate(priors_max_xy, axis=0)
        self.priors_variances = np.concatenate(priors_variances, axis=0)
        self.priors = np.concatenate(priors, axis=0)
        self.map_offsets = map_offsets

        # normalized prior boxes
        image_h, image_w = self.image_size
        self.priors_xy_norm = self.priors_xy / (image_w, image_h)
        self.priors_wh_norm = self.priors_wh / (image_w, image_h)
        self.priors_min_xy_norm = self.priors_min_xy / (image_w, image_h)
        self.priors_max_xy_norm = self.priors_max_xy / (image_w, image_h)
        self.priors_norm = np.concatenate(
            [self.priors_min_xy_norm, self.priors_max_xy_norm, self.priors_variances],
            axis=1,
        )

    def decode(
        self, model_output, confidence_threshold=0.01, keep_top_k=200, sparse=True
    ):

        prior_mask = model_output[:, 12:] > confidence_threshold

        if sparse:
            # compute boxes only if the confidence is high enough and the class is not background
            mask = np.any(prior_mask[:, 1:], axis=1)
            prior_mask = prior_mask[mask]
            mask = np.ix_(mask)[0]
            model_output = model_output[mask]
            priors_xy = self.priors_xy[mask] / self.image_size
            priors_wh = self.priors_wh[mask] / self.image_size
            priors_variances = self.priors_variances[mask, :]
        else:
            priors_xy = self.priors_xy / self.image_size
            priors_wh = self.priors_wh / self.image_size
            priors_variances = self.priors_variances

        offsets = model_output[:, :4]
        offsets_quads = model_output[:, 4:12]
        confidence = model_output[:, 12:]

        priors_xy_minmax = np.hstack(
            [priors_xy - priors_wh / 2, priors_xy + priors_wh / 2]
        )
        ref = priors_xy_minmax[:, (0, 1, 2, 1, 2, 3, 0, 3)]  # corner points
        variances_xy = priors_variances[:, 0:2]
        variances_wh = priors_variances[:, 2:4]

        num_priors = offsets.shape[0]
        num_classes = confidence.shape[1]

        # compute bounding boxes from local offsets
        boxes = np.empty((num_priors, 4))
        offsets = offsets * priors_variances
        boxes_xy = priors_xy + offsets[:, 0:2] * priors_wh
        boxes_wh = priors_wh * np.exp(offsets[:, 2:4])
        boxes[:, 0:2] = boxes_xy - boxes_wh / 2.0  # xmin, ymin
        boxes[:, 2:4] = boxes_xy + boxes_wh / 2.0  # xmax, ymax
        boxes = np.clip(boxes, 0.0, 1.0)

        # do non maximum suppression
        results = []
        for c in range(1, num_classes):
            mask = prior_mask[:, c]
            boxes_to_process = boxes[mask]
            if len(boxes_to_process) > 0:
                confs_to_process = confidence[mask, c]

                idx = non_maximum_suppression(
                    boxes_to_process[:, :4],
                    confs_to_process,
                    self.nms_thresh,
                    self.nms_top_k,
                )

                good_boxes = boxes_to_process[idx]
                good_confs = confs_to_process[idx][:, None]
                labels = np.ones((len(idx), 1)) * c

                good_quads = ref[mask][idx] + offsets_quads[mask][idx] * np.tile(
                    priors_wh[mask][idx], (1, 4)
                ) * np.tile(variances_xy[mask][idx], (1, 4))

                c_pred = np.concatenate(
                    (good_boxes, good_quads, good_confs, labels), axis=1
                )
                results.extend(c_pred)
        if len(results) > 0:
            results = np.array(results)
            order = np.argsort(-results[:, 12])
            results = results[order]
            results = results[:keep_top_k]
        else:
            results = np.empty((0, 6))
        self.results = results
        return results
